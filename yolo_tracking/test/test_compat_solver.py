import numpy as np
import pytest
from scipy.optimize import linear_sum_assignment

from yolo_tracking.trackers.compat import lapjv_cost_limit
from yolo_tracking.trackers.utils.matching import linear_assignment


def _assert_result(result, matches, unmatched_a, unmatched_b):
    actual_matches, actual_a, actual_b = result
    np.testing.assert_array_equal(actual_matches, np.asarray(matches, dtype=int).reshape(-1, 2))
    np.testing.assert_array_equal(actual_a, np.asarray(unmatched_a, dtype=int))
    np.testing.assert_array_equal(actual_b, np.asarray(unmatched_b, dtype=int))
    if len(actual_matches):
        assert np.all(np.diff(actual_matches[:, 0]) > 0)


def test_square_assignment_and_row_order():
    cost = np.array([[0.1, 0.9], [0.8, 0.2]], dtype=np.float32)
    _assert_result(lapjv_cost_limit(cost, 0.5), [[0, 0], [1, 1]], [], [])


def test_matches_are_returned_in_ascending_row_order():
    cost = np.array([[0.9, 0.1], [0.1, 0.9]], dtype=np.float32)
    _assert_result(lapjv_cost_limit(cost, 0.5), [[0, 1], [1, 0]], [], [])


def test_rectangular_assignment():
    cost = np.array([[0.1, 0.8, 0.9], [0.7, 0.2, 0.6]], dtype=np.float64)
    _assert_result(lapjv_cost_limit(cost, 0.5), [[0, 0], [1, 1]], [], [2])


@pytest.mark.parametrize("shape", [(0, 0), (0, 3), (4, 0)])
def test_empty_assignment(shape):
    _assert_result(lapjv_cost_limit(np.empty(shape), 0.5), [], range(shape[0]), range(shape[1]))


def test_all_costs_above_limit_are_unmatched():
    cost = np.full((2, 3), 0.9, dtype=np.float32)
    _assert_result(lapjv_cost_limit(cost, 0.5), [], [0, 1], [0, 1, 2])


def test_cost_limit_is_part_of_the_optimization():
    cost = np.array([[0.45, 0.001], [100.0, 0.46]])
    padded = lapjv_cost_limit(cost, 0.5)
    _assert_result(padded, [[0, 1]], [1], [0])

    rows, cols = linear_sum_assignment(cost)
    solve_then_filter = np.asarray(
        [[row, col] for row, col in zip(rows, cols) if cost[row, col] <= 0.5],
        dtype=int,
    )
    assert set(map(tuple, solve_then_filter)) == {(0, 0), (1, 1)}
    assert set(map(tuple, padded[0])) != set(map(tuple, solve_then_filter))


def test_matching_keeps_its_degenerate_fast_path():
    matches, unmatched_a, unmatched_b = linear_assignment(np.empty((2, 0)), 0.5)
    assert matches.shape == (0, 2)
    assert unmatched_a == (0, 1)
    assert unmatched_b == ()
