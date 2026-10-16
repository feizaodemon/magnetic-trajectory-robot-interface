"""Characterization tests for the point-parsing helpers.

Six helpers turn a point into ``(x, y)`` floats, each with slightly different
rules (u/v keys, nested ``point``, finite checks, payload key order). These
tables pin what each one returns today, so a refactor that shares code
between them cannot change behaviour without failing here. Some rows pin
behaviour that looks accidental (for example ``coerce_xy_point`` keeps NaN);
changing those is a separate, deliberate decision.
"""

import math

import pytest

from colmag_ros import (
    dashboard_points,
    dtw_dataset_io,
    dtw_template_bank_tools,
    trajectory_rasterizer,
    trajectory_strokes,
)
from magnetic_ui_state_node import normalize_capture_points

nan = math.nan
inf = math.inf

POINT_FUNCTIONS = (
    dtw_dataset_io.normalize_point,
    lambda p: dtw_template_bank_tools.coerce_points([p]),
    trajectory_strokes.coerce_point,
    dashboard_points.coerce_xy_point,
    lambda p: trajectory_rasterizer._coerce_finite_points([p]),
    lambda p: normalize_capture_points([p]),
)

PAYLOAD_FUNCTIONS = (
    dtw_template_bank_tools.coerce_points,
    dtw_dataset_io.normalize_points,
    dashboard_points.extract_xy_points,
)

POINT_INPUTS = {
    'pair': (1, 2),
    'list_str': [1.5, '2.5'],
    'str_pair': ('1', '2'),
    'triple': (1, 2, 3),
    'len1': [1],
    'empty_list': [],
    'str12': '12',
    'str_ab': 'ab',
    'none': None,
    'int': 5,
    'bool_pair': (True, False),
    'xy': {'x': 1, 'y': 2},
    'xy_str': {'x': '1', 'y': '2'},
    'uv': {'u': 1, 'v': 2},
    'x_v': {'x': 1, 'v': 2},
    'x_only': {'x': 1},
    'y_only': {'y': 2},
    'x_none': {'x': None, 'y': 2},
    'xy_bad': {'x': 'a', 'y': 1},
    'empty_dict': {},
    'position_xy': {'position': {'x': 1, 'y': 2}},
    'position_uv': {'position': {'u': 1, 'v': 2}},
    'position_x_only': {'position': {'x': 1}},
    'point_xy': {'point': {'x': 1, 'y': 2}},
    'point_uv': {'point': {'u': 1, 'v': 2}},
    'xy_and_position': {'x': 1, 'y': 2, 'position': {'x': 5, 'y': 6}},
    'nan_pair': (nan, 1),
    'inf_pair': (inf, 1),
    'neg_inf_pair': (1, -inf),
    'nan_dict': {'x': nan, 'y': 1},
    'nan_str_dict': {'x': 'nan', 'y': 1},
    'inf_str_dict': {'x': 'inf', 'y': '1'},
}

PAYLOAD_INPUTS = {
    'points_list': {'points': [(1, 2)], 'trajectory': [(3, 4)]},
    'trajectory_list': {'trajectory': [(3, 4)], 'points': [(1, 2)]},
    'points_none_then_path': {'points': None, 'path': [(5, 6)]},
    'points_dict_then_path': {'points': {'x': 1, 'y': 2}, 'path': [(5, 6)]},
    'points_tuple': {'points': ((1, 2),)},
    'points_2d': {'points_2d': [{'position': {'x': 7, 'y': 8}}]},
    'no_key': {'strokes': [[(1, 2)]]},
    'bare_list': [(1, 2), None, 'bad', {'u': 3, 'v': 4}],
    'none': None,
}


# input name -> results of, in order: normalize_point, coerce_points([p]),
# coerce_point, coerce_xy_point, _coerce_finite_points([p]),
# normalize_capture_points([p]).
# Each result is repr() of the return value, or 'raises <Exception>'.
POINT_CASES = {
    'pair': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '[[0.0, 0.0]]',
    ),
    'list_str': (
        '(1.5, 2.5)',
        '[(1.5, 2.5)]',
        '(1.5, 2.5)',
        '(1.5, 2.5)',
        '[(1.5, 2.5)]',
        '[[0.0, 0.0]]',
    ),
    'str_pair': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '[[0.0, 0.0]]',
    ),
    'triple': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '[[0.0, 0.0]]',
    ),
    'len1': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'empty_list': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'str12': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        'None',
        '[(1.0, 2.0)]',
        '[]',
    ),
    'str_ab': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'none': (
        'None',
        '[]',
        'None',
        'None',
        'raises TypeError',
        '[]',
    ),
    'int': (
        'None',
        '[]',
        'None',
        'None',
        'raises TypeError',
        '[]',
    ),
    'bool_pair': (
        '(1.0, 0.0)',
        '[(1.0, 0.0)]',
        '(1.0, 0.0)',
        '(1.0, 0.0)',
        '[(1.0, 0.0)]',
        '[[0.0, 0.0]]',
    ),
    'xy': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        'raises KeyError',
        '[]',
    ),
    'xy_str': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        'raises KeyError',
        '[]',
    ),
    'uv': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        'None',
        'None',
        'raises KeyError',
        '[]',
    ),
    'x_v': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        'None',
        'None',
        'raises KeyError',
        '[]',
    ),
    'x_only': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'y_only': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'x_none': (
        'None',
        '[]',
        'None',
        'None',
        'raises KeyError',
        '[]',
    ),
    'xy_bad': (
        'None',
        '[]',
        'None',
        'None',
        'raises KeyError',
        '[]',
    ),
    'empty_dict': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'position_xy': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        '[]',
        '[]',
    ),
    'position_uv': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'position_x_only': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'point_xy': (
        'None',
        '[]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        '[]',
        '[]',
    ),
    'point_uv': (
        'None',
        '[]',
        'None',
        'None',
        '[]',
        '[]',
    ),
    'xy_and_position': (
        '(1.0, 2.0)',
        '[(1.0, 2.0)]',
        '(1.0, 2.0)',
        '(1.0, 2.0)',
        'raises KeyError',
        '[]',
    ),
    'nan_pair': (
        'None',
        '[]',
        'None',
        '(nan, 1.0)',
        '[]',
        '[[0.0, 0.0]]',
    ),
    'inf_pair': (
        'None',
        '[]',
        'None',
        '(inf, 1.0)',
        '[]',
        '[[0.0, 0.0]]',
    ),
    'neg_inf_pair': (
        'None',
        '[]',
        'None',
        '(1.0, -inf)',
        '[]',
        '[[0.0, 0.0]]',
    ),
    'nan_dict': (
        'None',
        '[]',
        'None',
        '(nan, 1.0)',
        'raises KeyError',
        '[]',
    ),
    'nan_str_dict': (
        'None',
        '[]',
        'None',
        '(nan, 1.0)',
        'raises KeyError',
        '[]',
    ),
    'inf_str_dict': (
        'None',
        '[]',
        'None',
        '(inf, 1.0)',
        'raises KeyError',
        '[]',
    ),
}

# payload name -> results of, in order: coerce_points, normalize_points,
# extract_xy_points.
PAYLOAD_CASES = {
    'points_list': ('[(1.0, 2.0)]', '[(1.0, 2.0)]', '[(3.0, 4.0)]'),
    'trajectory_list': ('[(1.0, 2.0)]', '[(1.0, 2.0)]', '[(3.0, 4.0)]'),
    'points_none_then_path': ('[]', '[(5.0, 6.0)]', '[(5.0, 6.0)]'),
    'points_dict_then_path': ('[]', '[(5.0, 6.0)]', '[(5.0, 6.0)]'),
    'points_tuple': ('[(1.0, 2.0)]', '[]', '[]'),
    'points_2d': ('[(7.0, 8.0)]', '[(7.0, 8.0)]', '[(7.0, 8.0)]'),
    'no_key': ('[]', '[]', '[]'),
    'bare_list': ('[(1.0, 2.0), (3.0, 4.0)]', '[(1.0, 2.0), (3.0, 4.0)]', '[]'),
    'none': ('[]', '[]', '[]'),
}


def _result(fn, value):
    try:
        return repr(fn(value))
    except Exception as exc:  # pinned as part of the behaviour
        return "raises " + type(exc).__name__


@pytest.mark.parametrize("name", sorted(POINT_INPUTS))
def test_point_helpers_keep_their_results(name):
    got = tuple(_result(fn, POINT_INPUTS[name]) for fn in POINT_FUNCTIONS)
    assert got == POINT_CASES[name]


@pytest.mark.parametrize("name", sorted(PAYLOAD_INPUTS))
def test_payload_helpers_keep_their_results(name):
    got = tuple(_result(fn, PAYLOAD_INPUTS[name]) for fn in PAYLOAD_FUNCTIONS)
    assert got == PAYLOAD_CASES[name]


def test_tables_cover_the_same_inputs():
    assert sorted(POINT_INPUTS) == sorted(POINT_CASES)
    assert sorted(PAYLOAD_INPUTS) == sorted(PAYLOAD_CASES)
