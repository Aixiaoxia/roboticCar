"""Conservative image-space proximity checks, not full footprint collision checking."""
import numpy as np


def assess_depth(depth_m, stop_distance=0.6, minimum_valid_fraction=0.5,
                 minimum_tile_valid_fraction=0.15, check_blind_sectors=True):
    if depth_m.ndim != 2 or min(depth_m.shape) < 12:
        return False, 'invalid_dimensions', 0.0, None
    # Full horizontal field, exclude only top 10% and bottom 15%.
    roi = depth_m[int(.10*depth_m.shape[0]):int(.85*depth_m.shape[0]), :]
    valid = np.isfinite(roi) & (roi >= .1) & (roi <= 8.0)
    fraction = float(valid.mean())
    nearest = float(roi[valid].min()) if valid.any() else None
    if nearest is not None and nearest < stop_distance:
        return False, 'near_obstacle', fraction, nearest
    if fraction < minimum_valid_fraction:
        return False, 'insufficient_depth', fraction, nearest
    # Reject sectors with missing measurements, even if the whole-image mean is high.
    if check_blind_sectors:
        for rows in np.array_split(valid, 3, axis=0):
            for tile in np.array_split(rows, 4, axis=1):
                if float(tile.mean()) < minimum_tile_valid_fraction:
                    return False, 'blind_sector', fraction, nearest
    return True, 'clear_in_depth_view', fraction, nearest
