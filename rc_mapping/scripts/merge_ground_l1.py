import argparse
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
import yaml


def read_map(metadata_path):
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    image_path = metadata_path.parent / metadata['image']
    pixels = np.array(Image.open(image_path).convert('L'))
    if not set(np.unique(pixels)).issubset({0, 205, 254}):
        raise ValueError(f'Expected map_saver trinary pixels 0/205/254: {image_path}')
    if metadata.get('negate', 0) != 0:
        raise ValueError('Negated source maps are not supported.')
    return metadata, pixels, image_path


def fit_rigid_transform(source, target):
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 2:
        raise ValueError('Corresponding landmarks must have matching Nx2 shapes.')
    if len(source) < 3 or np.linalg.matrix_rank(source - source.mean(axis=0)) < 2:
        raise ValueError('At least three non-collinear landmarks are required.')
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    left, singular, right = np.linalg.svd(
        (source - source_center).T @ (target - target_center)
    )
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0:
        right[-1] *= -1
        rotation = right.T @ left.T
    translation = target_center - rotation @ source_center
    return np.column_stack([rotation, translation])


def pixel_to_world(metadata, height):
    origin_x, origin_y, yaw = metadata['origin']
    resolution = metadata['resolution']
    cosine, sine = math.cos(yaw), math.sin(yaw)
    origin_rotation = np.array([[cosine, -sine], [sine, cosine]])
    affine = np.eye(3)
    affine[:2, :2] = origin_rotation @ np.diag([resolution, -resolution])
    affine[:2, 2] = np.array([origin_x, origin_y]) + origin_rotation @ np.array([
        resolution * 0.5, resolution * (height - 0.5),
    ])
    return affine


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save_preview(path, ground, warped, selected, regions):
    colors = np.repeat(ground[:, :, None], 3, axis=2)
    colors[ground == 205] = [180, 180, 180]
    colors[selected & (warped == 254)] = [150, 215, 250]
    colors[selected & (warped == 0)] = [230, 65, 25]
    colors[(ground == 0) & selected & (warped == 254)] = [180, 0, 180]
    scale = 3
    preview = Image.fromarray(colors).resize(
        (ground.shape[1] * scale, ground.shape[0] * scale), Image.Resampling.NEAREST,
    )
    canvas = Image.new('RGB', (preview.width + 360, preview.height + 50), '#303030')
    canvas.paste(preview, (0, 50))
    painter = ImageDraw.Draw(canvas)
    painter.text((12, 15), 'GROUND + L1 / ALIGNMENT DRAFT', fill='white')
    labels = [
        ('Black: original ground obstacles', 'white'),
        ('Blue: selected L1 free space', '#96d7fa'),
        ('Orange: selected L1 obstacles', '#e64119'),
        ('Purple: conflicting observations', '#d040d0'),
        ('Gray: unknown / unobserved', '#b4b4b4'),
        ('RAMP / STAIRS: review locations', '#ffbb00'),
        ('Original obstacles are preserved.', 'white'),
        ('No connector was painted free.', 'white'),
        ('Resolution: 0.05 m per cell', 'white'),
    ]
    for index, (label, color) in enumerate(labels):
        painter.text((preview.width + 12, 60 + index * 30), label, fill=color)
    for name, polygon in regions.items():
        outline = [(point[0] * scale, point[1] * scale + 50) for point in polygon]
        painter.line(outline + [outline[0]], fill='#ffbb00', width=2)
        painter.text((outline[0][0] + 5, outline[0][1] + 5), name.upper(), fill='#ff8800')
    canvas.save(path)


def merge(config_path):
    config = json.loads(config_path.read_text(encoding='utf-8'))
    output_dir = config_path.parent
    ground_path = (output_dir / config['ground_map']).resolve()
    l1_path = (output_dir / config['l1_map']).resolve()
    ground_metadata, ground, ground_image = read_map(ground_path)
    l1_metadata, l1, l1_image = read_map(l1_path)
    if not math.isclose(ground_metadata['resolution'], l1_metadata['resolution']):
        raise ValueError('Source resolutions differ; explicit resampling is required.')
    input_paths = [ground_path, l1_path, ground_image, l1_image]
    initial_hashes = {str(path): sha256(path) for path in input_paths}
    source_points = np.asarray(config['landmarks_l1_pixels'], dtype=float)
    target_points = np.asarray(config['landmarks_ground_pixels'], dtype=float)
    matrix = fit_rigid_transform(source_points, target_points)
    errors = np.linalg.norm(
        source_points @ matrix[:, :2].T + matrix[:, 2] - target_points, axis=1,
    )
    if errors.max() > 3:
        raise ValueError(f'Landmark alignment exceeds three cells: {errors.tolist()}')
    region = np.asarray(config['l1_region_pixels'], dtype=np.int32)
    region_mask = np.zeros_like(l1, dtype=np.uint8)
    cv2.fillPoly(region_mask, [region], 255)
    transformed_region = region @ matrix[:, :2].T + matrix[:, 2]
    if (transformed_region.min(axis=0) < 0).any() or (
        transformed_region.max(axis=0) > np.array([ground.shape[1] - 1, ground.shape[0] - 1])
    ).any():
        raise ValueError('Selected L1 region extends beyond ground canvas.')
    dimensions = (ground.shape[1], ground.shape[0])
    warped = cv2.warpAffine(
        l1, matrix, dimensions, flags=cv2.INTER_NEAREST, borderValue=205,
    )
    selected = cv2.warpAffine(
        region_mask, matrix, dimensions, flags=cv2.INTER_NEAREST, borderValue=0,
    ) > 0
    merged = ground.copy()
    add_free = selected & (warped == 254) & (ground == 205)
    add_occupied = selected & (warped == 0)
    merged[add_free] = 254
    merged[add_occupied] = 0
    if not np.all(merged[ground == 0] == 0):
        raise AssertionError('Original ground obstacles were lost.')
    if not np.array_equal(merged[~selected], ground[~selected]):
        raise AssertionError('Pixels outside selected L1 region changed.')
    Image.fromarray(merged).save(output_dir / 'map.pgm')
    Image.fromarray(merged).resize(
        (merged.shape[1] * 3, merged.shape[0] * 3), Image.Resampling.NEAREST,
    ).save(output_dir / 'map_preview.png')
    metadata = dict(ground_metadata)
    metadata.update(image='map.pgm', mode='trinary', free_thresh=0.196, occupied_thresh=0.65)
    (output_dir / 'map.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8',
    )
    probabilities = (255 - merged.astype(float)) / 255
    if np.any(probabilities[merged == 205] < metadata['free_thresh']):
        raise AssertionError('Unknown gray pixels would load as free space.')
    save_preview(
        output_dir / 'alignment_preview.png', ground, warped, selected,
        config.get('ground_review_regions', {}),
    )
    l1_preview = np.repeat(l1[:, :, None], 3, axis=2)
    l1_preview[region_mask == 0] = (l1_preview[region_mask == 0] * 0.3).astype(np.uint8)
    cv2.polylines(l1_preview, [region], True, (0, 160, 255), 1)
    Image.fromarray(l1_preview).resize(
        (l1.shape[1] * 3, l1.shape[0] * 3), Image.Resampling.NEAREST,
    ).save(output_dir / 'l1_region_preview.png')
    pixel_affine = np.eye(3)
    pixel_affine[:2] = matrix
    world_affine = (
        pixel_to_world(ground_metadata, ground.shape[0]) @ pixel_affine
        @ np.linalg.inv(pixel_to_world(l1_metadata, l1.shape[0]))
    )
    report = {
        'status': 'draft_requires_connector_review',
        'source_sha256': initial_hashes,
        'resolution_m': ground_metadata['resolution'],
        'pixel_transform_l1_to_ground': matrix.tolist(),
        'image_rotation_deg': math.degrees(math.atan2(matrix[1, 0], matrix[0, 0])),
        'world_transform_l1_to_ground': world_affine.tolist(),
        'world_yaw_deg': math.degrees(math.atan2(world_affine[1, 0], world_affine[0, 0])),
        'landmark_errors_m': (errors * ground_metadata['resolution']).tolist(),
        'landmark_rms_error_m': float(np.sqrt(np.mean(errors**2)) * ground_metadata['resolution']),
        'added_free_cells': int(add_free.sum()),
        'added_occupied_cells': int((add_occupied & (ground != 0)).sum()),
        'ground_occupied_l1_free_conflicts': int((selected & (warped == 254) & (ground == 0)).sum()),
        'ground_free_l1_occupied_conflicts': int((selected & (warped == 0) & (ground == 254)).sum()),
        'fusion': 'Occupied wins; L1 free fills ground unknown inside annotated L1 region only.',
        'connectors': 'No ramp/stair obstacle removal or free-space interpolation performed.',
    }
    (output_dir / 'report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8',
    )
    if {str(path): sha256(path) for path in input_paths} != initial_hashes:
        raise AssertionError('Source map files changed.')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Merge selected L1 observations into the ground map.')
    parser.add_argument('alignment', type=Path)
    arguments = parser.parse_args()
    merge(arguments.alignment.resolve())
