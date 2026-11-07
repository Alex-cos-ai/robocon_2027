import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import yaml


def export_map(config_path):
    config = json.loads(config_path.read_text(encoding='utf-8'))
    output_dir = config_path.parent
    base_yaml = (output_dir / config['base_map']).resolve()
    metadata = yaml.safe_load(base_yaml.read_text(encoding='utf-8'))
    base_image = base_yaml.parent / metadata['image']
    base = np.array(Image.open(base_image).convert('L'))
    source_hashes = {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (base_yaml, base_image)
    }
    resolution = float(metadata['resolution'])
    width = float(config['width_m'])
    length = float(config['horizontal_length_m'])
    transition = float(config['bottom_transition_m'])
    if resolution <= 0 or width <= 0 or length <= 0 or transition < 0:
        raise ValueError('Invalid ramp dimensions.')
    bottom = np.asarray(config['bottom_center_pixel'], dtype=float)
    direction = np.asarray(config['uphill_direction_image'], dtype=float)
    if not np.isfinite(direction).all() or np.linalg.norm(direction) == 0:
        raise ValueError('Ramp direction must be finite and nonzero.')
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    top = bottom + direction * length / resolution
    rows, columns = np.indices(base.shape)
    relative = np.stack([columns - bottom[0], rows - bottom[1]], axis=-1)
    along = relative @ direction
    across = relative @ normal
    ramp = (along >= 0) & (along <= length / resolution) & (abs(across) < width / resolution / 2)
    approach = (along >= -transition / resolution) & (along < 0) & (abs(across) < width / resolution / 2)
    modified_region = ramp | approach
    merged = base.copy()
    merged[modified_region] = 254
    if not np.array_equal(merged[~modified_region], base[~modified_region]):
        raise AssertionError('Unexpected changes outside ramp and bottom transition.')
    if set(np.unique(merged)) - {0, 205, 254}:
        raise AssertionError('Output is not a plain trinary occupancy map.')
    component_count, labels = cv2.connectedComponents((merged == 254).astype(np.uint8), connectivity=4)
    component_ids = {
        name: int(labels[point[1], point[0]])
        for name, point in config['verification_pixels'].items()
    }
    if 0 in component_ids.values() or len(set(component_ids.values())) != 1:
        raise AssertionError(f'Ground, ramp and L1 are not connected: {component_ids}')
    Image.fromarray(merged).save(output_dir / 'map.pgm')
    Image.fromarray(merged).resize(
        (merged.shape[1] * 3, merged.shape[0] * 3), Image.Resampling.NEAREST,
    ).save(output_dir / 'map.png')
    metadata.update(image='map.pgm', mode='trinary', free_thresh=0.196, occupied_thresh=0.65)
    (output_dir / 'map.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    half_width = normal * width / resolution / 2
    report = {
        'status': 'plain_occupancy_map_with_manually_modelled_ramp',
        'base_sha256': source_hashes,
        'resolution_m': resolution,
        'ramp_width_m': width,
        'ramp_horizontal_length_m': length,
        'top_platform_included_in_ramp_length': False,
        'ramp_bottom_center_pixel': bottom.tolist(),
        'ramp_top_center_pixel': top.tolist(),
        'ramp_polygon_pixels': np.array([
            bottom - half_width, bottom + half_width, top + half_width, top - half_width,
        ]).tolist(),
        'bottom_transition_m': transition,
        'changed_cells': int(np.count_nonzero(merged != base)),
        'occupied_cells_cleared_within_ramp_and_approach': int(np.count_nonzero(modified_region & (base == 0))),
        'four_neighbor_connectivity': component_ids,
        'stairs': 'Unchanged; no stair connection was cleared.',
        'placement_note': config['placement'],
        'limitation': 'Connectivity check uses a point robot; physical placement and robot clearance require verification. No height or slope information is encoded in the occupancy map.',
    }
    (output_dir / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    for filename, digest in source_hashes.items():
        if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != digest:
            raise AssertionError(f'Base map changed: {filename}')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Export plain ground/L1 occupancy map with a measured ramp.')
    parser.add_argument('config', type=Path)
    arguments = parser.parse_args()
    export_map(arguments.config.resolve())
