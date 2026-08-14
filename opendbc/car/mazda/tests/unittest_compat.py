"""unittest helpers so Mazda tests run under CI's unittest-parallel runner.

CI does not install pytest. These stand in for pytest.mark.parametrize and
pytest.approx without changing assertion semantics.
"""
from __future__ import annotations

import functools


class approx:
  def __init__(self, expected, abs=None, rel=None):  # noqa: A002
    self.expected = expected
    self.abs = 1e-6 if abs is None else abs
    self.rel = rel

  def __eq__(self, other):
    tol = self.abs
    if self.rel is not None:
      tol = max(tol, abs(self.expected) * self.rel)
    return abs(other - self.expected) <= tol

  def __repr__(self):
    return f"approx({self.expected!r})"


def _combos(argnames, argvalues):
  if isinstance(argnames, str):
    names = [n.strip() for n in argnames.strip("()").split(",") if n.strip()]
  else:
    names = list(argnames)
  out = []
  for vals in argvalues:
    if len(names) == 1:
      if isinstance(vals, tuple) and len(vals) == 1:
        vals = vals[0]
      out.append({names[0]: vals})
    else:
      if not isinstance(vals, (list, tuple)):
        raise TypeError(f"expected tuple for {names}, got {vals!r}")
      out.append(dict(zip(names, vals, strict=True)))
  return out


def parametrize(argnames, argvalues):
  """Apply value sets via unittest subTest. Stackable; class-level wraps test_* methods."""
  cases = _combos(argnames, argvalues)

  def decorator(fn):
    if isinstance(fn, type):
      for name, method in list(vars(fn).items()):
        if name.startswith("test_") and callable(method):
          setattr(fn, name, decorator(method))
      return fn

    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
      for combo in cases:
        merged = {**kwargs, **combo}
        with self.subTest(**combo):
          fn(self, *args, **merged)
    return wrapper

  return decorator
