# Browser recognizer demo

A static page where a visitor draws a symbol and sees how the runtime DTW
template-bank recognizer ranks it, with the same templates and acceptance
gates as the robot side. It is published to GitHub Pages by
[`.github/workflows/pages.yml`](../../.github/workflows/pages.yml).

| File | Role |
| --- | --- |
| `recognizer.js` | JavaScript port of the runtime path: preprocessing and DTW from `colmag_ros/src/colmag_ros/dtw_bank_features.py`, ranking and gates from `recognize_with_template_bank()` in `dtw_template_bank_tools.py`. No DOM code. |
| `app.js` | Drawing pad, result panel and template gallery. |
| `index.html`, `style.css` | The page. Light and dark themes, works at phone width. |

The page loads two generated files. `scripts/build_web_demo.py` writes them
next to a copy of the page: `bank.json` (the tracked bank
`colmag_ros/config/dtw_banks/m104c1_8symbol_mouse_seeded_yflip.json`) and
`config.json` (the gate defaults from `dtw_template_bank_tools.py` and the
symbol names from `symbol_semantics.py`). So the page cannot drift from the
Python values.

## Run locally

```bash
python3 scripts/build_web_demo.py          # writes outputs/web_demo/
python3 -m http.server -d outputs/web_demo 8000
# open http://localhost:8000/
```

## How it is checked

`tests/test_web_recognizer_parity.py` runs `recognizer.js` under Node and the
Python runtime function on the same seeded queries (exact, noisy, rotated,
truncated and random strokes, so both accepted and rejected results occur) and
requires the same labels, template ids, distances, confidences and gate
decisions. CI sets `COLMAG_REQUIRE_NODE=1` so the test cannot silently skip.

## What it does not show

- The canvas y axis points down; the page flips it so queries are y-up, the
  convention the runtime expects. The magnetic board's own capture path and its
  noise are not part of this page.
- The templates are one person's mouse drawings, so other handwriting is
  rejected more often than in the README's leave-one-out numbers.
- Nothing here talks to ROS or a robot; on the real system an accepted symbol
  still waits for operator confirmation.
