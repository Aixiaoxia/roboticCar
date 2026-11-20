#!/usr/bin/env python3
"""Offline identification of delayed first/second order BUNKER speed responses."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def response(t, p, order):
    x = np.maximum(t-p[-1], 0)
    if order == 1:
        h = -np.expm1(-x/p[1])
    else:
        wn, zeta = p[1:3]
        if abs(zeta-1) < 1e-5:
            h = 1-(1+wn*x)*np.exp(-wn*x)
        else:
            root = np.sqrt(complex(zeta*zeta-1))
            r1, r2 = wn*(-zeta+root), wn*(-zeta-root)
            h = (1+(r2*np.exp(r1*x)-r1*np.exp(r2*x))/(r1-r2)).real
    return p[0]*h*(t >= p[-1])


def predict(run, p, order):
    return response(run['t'][:, None]-run['edges'][None, :], p, order) @ run['delta']


def load(path, axis):
    meta = json.loads((path/'metadata.json').read_text())
    if meta.get('result') != 'completed' or meta['axis'] != axis:
        return None
    c = np.genfromtxt(path/'commands.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
    f = np.genfromtxt(path/'feedback.csv', delimiter=',', names=True)
    u = c['linear_x' if axis == 'linear' else 'angular_z'].astype(float)
    delta = np.diff(np.r_[0., u])
    changed = np.abs(delta) > 1e-10
    t = f['elapsed_s']; y = f['linear_velocity' if axis == 'linear' else 'angular_velocity']
    mask = (t >= c['elapsed_s'][0]) & (t <= c['elapsed_s'][-1]) & np.isfinite(y)
    t, y = t[mask], y[mask]
    if len(t) < 30 or not np.any(changed):
        raise ValueError(f'Insufficient data: {path}')
    edges = c['elapsed_s'][changed]
    baseline = y[t < edges[0]]
    if len(baseline) < 5 or abs(np.mean(baseline)) > .02:
        raise ValueError(f'Initial rest invalid: {path}')
    return dict(name=path.name, t=t, y=y, edges=edges, delta=delta[changed],
                amplitude=tuple(meta['amplitudes']), offset=float(np.mean(baseline)),
                dt=float(np.median(np.diff(t))))


def residual(runs, p, order):
    return np.concatenate([predict(r, p, order)-(r['y']-r['offset']) for r in runs])


def fit(runs, order):
    if order == 1:
        seeds = [[1, tau, delay] for tau in [.05, .2, .6] for delay in [0.01, .1, .3]]
        bounds = ([.01, .005, 0], [5, 10, 1.5])
    else:
        seeds = [[1, wn, zeta, delay] for wn in [3, 10, 30] for zeta in [.5, 1.2, 2]
                 for delay in [.01, .15]]
        bounds = ([.01, .1, .05, 0], [5, 200, 5, 1.5])
    best = min((least_squares(lambda p: residual(runs, p, order), seed,
                             bounds=bounds, max_nfev=400) for seed in seeds), key=lambda r: r.cost)
    return best.x, bounds


def metrics(runs, p, order):
    e = residual(runs, p, order)
    y = np.concatenate([r['y']-r['offset'] for r in runs])
    sse = float(e@e); n = len(e); k = len(p)
    return dict(rmse=float(np.sqrt(sse/n)), mae=float(np.mean(np.abs(e))),
                fit_percent=float(100*(1-np.linalg.norm(e)/np.linalg.norm(y-y.mean()))),
                bic=float(n*np.log(max(sse/n, 1e-30))+k*np.log(n)), n=n)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', type=Path, default=Path('data/system_identification'))
    ap.add_argument('--output', type=Path, default=Path('data/identification_results'))
    a = ap.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    report = {}; rows = []
    for axis in ['linear', 'angular']:
        runs = [r for p in sorted(a.data.glob('*/metadata.json')) if (r := load(p.parent, axis)) is not None]
        groups = {}
        for r in runs:
            groups.setdefault(r['amplitude'], []).append(r)
        # Last complete run at each amplitude is held out entirely, never used for fitting.
        train, valid = [], []
        for group in groups.values():
            if len(group) < 2:
                raise ValueError(f'{axis}: need >=2 independent runs per amplitude')
            train.extend(group[:-1]); valid.append(group[-1])
        if not train:
            raise ValueError(f'No usable {axis} runs')
        models = {}; params = {}
        for order in [1, 2]:
            p, bounds = fit(train, order); params[order] = p
            models[str(order)] = dict(parameters=dict(zip(['K', 'tau_s', 'delay_s'] if order == 1 else
                ['K', 'wn_rad_s', 'zeta', 'delay_s'], map(float, p))),
                training=metrics(train, p, order), validation=metrics(valid, p, order),
                near_bound=bool(np.any(np.isclose(p, bounds[0], rtol=.01, atol=1e-4)) or
                                np.any(np.isclose(p, bounds[1], rtol=.01, atol=1e-4))))
        # Demand at least 5% held-out RMSE reduction before recommending extra complexity.
        selected = 2 if models['2']['validation']['rmse'] < .95*models['1']['validation']['rmse'] else 1
        report[axis] = dict(models=models, recommended_order=selected,
                           training_runs=[r['name'] for r in train], validation_runs=[r['name'] for r in valid])
        fig, axs = plt.subplots(len(runs), 1, figsize=(11, 2.6*len(runs)), squeeze=False)
        for ax, r in zip(axs[:, 0], runs):
            t = r['t']; origin = r['edges'][0]
            u = (t[:, None] >= r['edges']) @ r['delta']
            ax.plot(t-origin, u, '--', label='command', color='gray')
            ax.plot(t-origin, r['y'], label='feedback', color='black', linewidth=1)
            for order in [1, 2]:
                yp = predict(r, params[order], order)+r['offset']
                ax.plot(t-origin, yp, label=f'order {order}')
                for ti, yi, pi in zip(t, r['y'], yp):
                    rows.append([axis, r['name'], 'validation' if r['name'] in {v['name'] for v in valid} else 'training', order, ti, yi, pi])
            ax.set_title(f"{'VALIDATION' if r['name'] in {v['name'] for v in valid} else 'TRAIN'} {r['name']}", fontsize=7)
            ax.set_ylabel('m/s' if axis == 'linear' else 'rad/s'); ax.grid(alpha=.3); ax.legend(fontsize=7)
        axs[-1, 0].set_xlabel('Time from first nonzero command (s)')
        fig.tight_layout(); fig.savefig(a.output/f'{axis}_comparison.png', dpi=150); plt.close(fig)
        print(axis, json.dumps(report[axis]['models'], indent=2), 'recommended:', selected)
    (a.output/'results.json').write_text(json.dumps(report, indent=2))
    with (a.output/'predictions.csv').open('w', newline='') as f:
        w = csv.writer(f); w.writerow(['axis', 'run', 'split', 'order', 'elapsed_s', 'measured', 'predicted']); w.writerows(rows)
    lines = ['# BUNKER speed response identification', '',
             'First order: G(s)=K exp(-Ls)/(tau s+1).',
             'Second order: G(s)=K wn² exp(-Ls)/(s²+2 zeta wn s+wn²).', '',
             'Models fit complete training runs; last run at each amplitude is held out.',
             'Recommendation requires >=5% validation RMSE improvement for second order.',
             'BIC assumes independent residuals; sampled feedback may be correlated, so it is supplementary.',
             'Delay includes transport/ROS scheduling. Output is chassis-reported speed, not independent ground truth.',
             'Baseline mean is subtracted per run. No time shift, smoothing or per-run gain adjustment is fitted.', '']
    for axis, result in report.items():
        lines += [f'## {axis}', '', f"Recommended order: {result['recommended_order']}", '',
                  '|Order|Parameters|Train RMSE|Validation RMSE|Validation fit %|Train BIC|',
                  '|---|---|---|---|---|---|']
        for order, m in result['models'].items():
            lines.append(f"|{order}|{m['parameters']}|{m['training']['rmse']:.6f}|{m['validation']['rmse']:.6f}|{m['validation']['fit_percent']:.2f}|{m['training']['bic']:.2f}|")
        lines += ['', 'Validation: '+', '.join(result['validation_runs']), '']
    (a.output/'report.md').write_text('\n'.join(lines))


if __name__ == '__main__':
    main()
