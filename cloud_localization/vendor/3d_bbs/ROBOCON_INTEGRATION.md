# Robocon integration provenance

Upstream: https://github.com/KOKIAOKI/3d_bbs

Pinned revision: `41529a34a2fb9618b5ff560fb3c2363f1615666d`.
MIT license and upstream copyright are retained in `LICENSE`.

The workspace builds the CPU and optionally CUDA core with its own modern
CMake targets (`src/robocon_cloud_localization/cmake/3d_bbs.cmake`). It does not
build upstream demos or install into system directories.

Local core extension, applied to both CPU and CUDA:

- Collect up to eight spatially distinct leaf poses (0.75 m / 0.35 rad NMS).
- Prune against 95% of the best occupancy count so near-best alternatives can
  reach the common GICP ambiguity/quality checks. The best transform is still
  updated only by an equal or better leaf. `set_candidate_score_ratio(1.0)`
  restores the original best-score pruning threshold.
- Expose `get_candidate_poses()` to the adapter. This bounded near-best set is
  not an exhaustive list of every possible ambiguous location.

Roto-translational branching, sparse hash voxelmaps and CUDA batched scoring
remain upstream algorithms. Timeouts still invalidate the result, rather than
returning an unfinished search as a globally verified solution. No node-count
instrumentation is claimed; the wrapper reports actual wall times and scores.

The patch against the pinned upstream commit is recorded in the workspace
`reports/relocalization_v2_bbs_20261101_201618/upstream_candidates.patch`.
