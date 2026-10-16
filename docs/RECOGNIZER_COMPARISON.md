# Recognizer comparison

The runtime recognizer is DTW against a template bank. This page compares it with five alternatives on the same data: its own Sakoe-Chiba window variant and four other single-stroke recognizers. It is an offline experiment: only DTW runs in the ROS pipeline, and nothing here changes what the robot does.

![Top-1 accuracy of each recognizer under synthetic perturbations](media/recognizer_robustness.svg)

## What is compared

All six are nearest-template classifiers with one interface in [`gesture_recognizers.py`](../colmag_ros/src/colmag_ros/gesture_recognizers.py). Each ranks the labels by their closest template, the same rule the runtime bank uses.

| Recognizer | Idea |
| --- | --- |
| DTW | The runtime method: centroid and max-abs normalization, 64-point arc-length resampling, dynamic time warping. |
| DTW (window 4) | The same, with the optional Sakoe-Chiba window that `dtw_template_window` enables at runtime. |
| Derivative DTW | DTW on the local slope of the stroke instead of its positions (Keogh and Pazzani, 2001). |
| Protractor ($1) | The closed-form member of the $1 family (Li, 2010). The best rotation is searched within ±45°, so `V` and an upside-down `A` stay distinct. |
| $P point cloud | Greedy point-cloud matching that ignores stroke direction and point order (Vatavu, Anthony and Wobbrock, 2012). 16 points. |
| Feature k-NN | 25 standardized geometric features: an 8-bin direction histogram, turning angle, closure, aspect ratio, start and end position, and 3×3 grid occupancy. |

All are pure Python, with no NumPy, so they run in the same Python 3.8 environment as the ROS nodes.

## Protocol

Leave-one-out over the 124 tracked templates (8 symbols): each template is held out once, the recognizer is fitted on the other 123, and the held-out stroke is classified. On clean input every method gets 100%, which says the bank separates well but cannot rank the methods. So each held-out stroke is also classified after four kinds of synthetic perturbation, at four strengths each, three seeded draws per strength:

- **Noise**: Gaussian jitter on every point, σ relative to the stroke's larger bounding-box side.
- **Rotation**: rotation about the centroid, clockwise or counter-clockwise.
- **Truncation**: the end of the stroke is cut off, as if the pen lifted early.
- **Stretch**: one axis, chosen at random, is scaled.

The perturbations are seeded per template, family, level and draw, so every recognizer sees exactly the same queries, and a rerun gives the same accuracy.

## Results
124 held-out queries clean and 372 per perturbed condition. Top-1 accuracy unless noted; the perturbed columns use each family's strongest level.

| Recognizer | Clean | Clean Top-3 | Noise σ=0.12 | Rotation 45° | Truncation 40% | Stretch 2.5× | Time per query |
| --- | --- | --- | --- | --- | --- | --- | --- |
| DTW | 100.0% | 100.0% | 95.2% | 84.1% | 16.9% | 86.8% | 89.0 ms |
| DTW (window 4) | 100.0% | 100.0% | 95.2% | 83.6% | 8.9% | 86.3% | 14.5 ms |
| Derivative DTW | 100.0% | 100.0% | 98.1% | 82.0% | 9.7% | 79.8% | 89.8 ms |
| Protractor ($1) | 100.0% | 100.0% | 100.0% | 100.0% | 9.7% | 91.1% | 0.4 ms |
| $P point cloud | 100.0% | 100.0% | 86.6% | 29.8% | 23.4% | 54.8% | 25.5 ms |
| Feature k-NN | 100.0% | 100.0% | 23.7% | 31.2% | 59.7% | 59.4% | 0.5 ms |

### Gaussian noise, sigma / stroke size

| Recognizer | σ=0.02 | σ=0.05 | σ=0.08 | σ=0.12 |
| --- | --- | --- | --- | --- |
| DTW | 100.0% | 100.0% | 98.7% | 95.2% |
| DTW (window 4) | 100.0% | 100.0% | 98.1% | 95.2% |
| Derivative DTW | 100.0% | 100.0% | 99.5% | 98.1% |
| Protractor ($1) | 100.0% | 100.0% | 100.0% | 100.0% |
| $P point cloud | 100.0% | 99.2% | 98.9% | 86.6% |
| Feature k-NN | 89.0% | 39.2% | 25.8% | 23.7% |

### Rotation, degrees

| Recognizer | 10° | 20° | 30° | 45° |
| --- | --- | --- | --- | --- |
| DTW | 100.0% | 100.0% | 100.0% | 84.1% |
| DTW (window 4) | 100.0% | 100.0% | 99.7% | 83.6% |
| Derivative DTW | 100.0% | 100.0% | 100.0% | 82.0% |
| Protractor ($1) | 100.0% | 100.0% | 100.0% | 100.0% |
| $P point cloud | 99.7% | 97.8% | 66.1% | 29.8% |
| Feature k-NN | 100.0% | 95.4% | 67.2% | 31.2% |

### Truncated stroke, fraction removed

| Recognizer | 10% | 20% | 30% | 40% |
| --- | --- | --- | --- | --- |
| DTW | 100.0% | 71.8% | 38.7% | 16.9% |
| DTW (window 4) | 100.0% | 66.9% | 28.2% | 8.9% |
| Derivative DTW | 99.2% | 77.4% | 61.3% | 9.7% |
| Protractor ($1) | 99.2% | 54.8% | 18.5% | 9.7% |
| $P point cloud | 99.2% | 71.0% | 39.5% | 23.4% |
| Feature k-NN | 100.0% | 82.3% | 65.3% | 59.7% |

### One-axis stretch, factor

| Recognizer | 1.25× | 1.5× | 2× | 2.5× |
| --- | --- | --- | --- | --- |
| DTW | 100.0% | 100.0% | 99.7% | 86.8% |
| DTW (window 4) | 100.0% | 100.0% | 99.7% | 86.3% |
| Derivative DTW | 100.0% | 100.0% | 85.2% | 79.8% |
| Protractor ($1) | 100.0% | 100.0% | 98.1% | 91.1% |
| $P point cloud | 100.0% | 98.9% | 80.6% | 54.8% |
| Feature k-NN | 100.0% | 98.1% | 86.8% | 59.4% |

Time per query is the mean on one CPU core of the machine that produced these numbers, against 123 templates. It depends on the machine.

## What the numbers say

- **Protractor is the most robust and by far the fastest** here: 100% under noise and rotation up to 45°, 91% at a 2.5× stretch, at 0.4 ms per query, about 200 times faster than DTW. Its rotation result is built in, because it searches rotations up to ±45°; it would drop beyond that.
- **DTW, the runtime method, is a solid second.** It stays at 95% or above under noise and at 84% or above under every rotation and stretch level. The 4-point window makes it about 6 times faster with the same accuracy, except on truncated strokes, where the narrow band cannot absorb the length mismatch (8.9% instead of 16.9% at 40% truncation).
- **A stroke that ends early is the weakness of every method.** With 30% of the stroke missing, the shape matchers fall to 19–61%. Feature k-NN degrades most gracefully (65% at 30%, 60% at 40%), because grid occupancy and direction features survive losing the tail. For the board, this points at the interaction design, the orientation clutch and the explicit `B Recognize`, as much as at the recognizer.
- **Hand-crafted features are fragile to noise**: Feature k-NN drops to 39% at σ=0.05, where every other method is above 99%. The direction histogram is computed on raw point-to-point steps, so jitter scrambles it.
- **$P loses to rotation and stretch** because its only normalization is scale and translation, and the point cloud carries no direction to compensate.

## Limits

- All 124 templates were drawn with a mouse by one person. The comparison ranks methods on that data; it is not an accuracy for other writers or for the magnetic board, which has never been part of this experiment.
- The perturbations are synthetic stand-ins for board noise, tilted drawing, early pen lifts and uneven scaling. Real board error is not known to look like any of them.
- Templates are compared with templates, so the queries come from the same distribution as the bank. Real queries would be harder.
- The DTW row normalizes the already-preprocessed bank strokes again, so its numbers can differ slightly from `scripts/eval_dtw_bank_loo.py`, which uses the stored points as they are.

## Reproduce

```bash
# About 10 minutes on four cores; writes nothing unless asked.
python3 scripts/compare_recognizers.py --repeats 3 --jobs 4 \
    --json-out docs/data/recognizer_comparison.json \
    --svg-out docs/media/recognizer_robustness.svg

# Re-render the tables and chart from the saved rows in a second.
python3 scripts/compare_recognizers.py --from-json docs/data/recognizer_comparison.json
```

The CI job `recognizer-compare` runs the comparison with one draw per strength on every pull request and puts the tables in its job summary. It reports only; the accuracy gate is in `dtw-eval`.
