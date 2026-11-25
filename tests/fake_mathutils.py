"""Minimal numpy stand-in for the parts of Blender's `mathutils` that the add-on uses.

Only for tests, so the camera pipeline can be exercised outside Blender. Conventions match
mathutils: Quaternion is (w, x, y, z); Matrix @ Matrix, Matrix @ Vector.
"""
import math
import numpy as np


class Vector:
    def __init__(self, v=(0.0, 0.0, 0.0)):
        self.v = np.array(list(v), dtype=float)

    x = property(lambda s: float(s.v[0]))
    y = property(lambda s: float(s.v[1]))
    z = property(lambda s: float(s.v[2]))

    def copy(self): return Vector(self.v)
    def __iter__(self): return iter(self.v.tolist())
    def __len__(self): return len(self.v)
    def __getitem__(self, i): return float(self.v[i])
    def __mul__(self, k): return Vector(self.v * k)
    __rmul__ = __mul__


class Quaternion:
    def __init__(self, q=(1.0, 0.0, 0.0, 0.0)):
        self.q = np.array(list(q), dtype=float)

    w = property(lambda s: float(s.q[0]))
    x = property(lambda s: float(s.q[1]))
    y = property(lambda s: float(s.q[2]))
    z = property(lambda s: float(s.q[3]))

    def __iter__(self): return iter(self.q.tolist())
    def __getitem__(self, i): return float(self.q[i])

    def to_matrix(self):
        w, x, y, z = self.q / np.linalg.norm(self.q)
        m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return Matrix(m)

    def to_euler(self, order="XYZ", compat=None):
        m = self.to_matrix().m
        # XYZ order, same as Blender: R = Rz * Ry * Rx
        sy = -m[2, 0]
        y = math.asin(max(-1.0, min(1.0, sy)))
        if abs(sy) < 0.999999:
            x = math.atan2(m[2, 1], m[2, 2])
            z = math.atan2(m[1, 0], m[0, 0])
        else:
            x = math.atan2(-m[1, 2], m[1, 1])
            z = 0.0
        if compat is not None:  # unwrap towards the previous angles
            def un(a, b):
                return a + 2 * math.pi * round((b - a) / (2 * math.pi))
            x, y, z = un(x, compat.x), un(y, compat.y), un(z, compat.z)
        return _Euler(x, y, z)


class _Euler:
    def __init__(self, x, y, z): self.x, self.y, self.z = x, y, z


class Matrix:
    def __init__(self, m=None):
        self.m = np.eye(4) if m is None else np.array(m, dtype=float)
        if self.m.shape == (3, 3):
            e = np.eye(4)
            e[:3, :3] = self.m
            self.m = e

    @staticmethod
    def Identity(n): return Matrix(np.eye(4))

    @staticmethod
    def Translation(v):
        m = np.eye(4)
        m[:3, 3] = list(v)
        return Matrix(m)

    @staticmethod
    def Rotation(angle, size, axis):
        c, s = math.cos(angle), math.sin(angle)
        r = {"X": [[1, 0, 0], [0, c, -s], [0, s, c]],
             "Y": [[c, 0, s], [0, 1, 0], [-s, 0, c]],
             "Z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
        return Matrix(r)

    def to_4x4(self): return Matrix(self.m)
    def inverted(self): return Matrix(np.linalg.inv(self.m))
    def copy(self): return Matrix(self.m)

    @property
    def translation(self): return Vector(self.m[:3, 3])

    @translation.setter
    def translation(self, v): self.m[:3, 3] = list(v)

    def decompose(self):
        r = self.m[:3, :3]
        t = float(np.trace(r))
        if t > 0:
            s = math.sqrt(t + 1.0) * 2
            q = [0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s]
        else:
            i = int(np.argmax(np.diag(r)))
            j, k = (i + 1) % 3, (i + 2) % 3
            s = math.sqrt(r[i, i] - r[j, j] - r[k, k] + 1.0) * 2
            qv = [0.0, 0.0, 0.0]
            qv[i] = 0.25 * s
            qv[j] = (r[j, i] + r[i, j]) / s
            qv[k] = (r[k, i] + r[i, k]) / s
            q = [(r[k, j] - r[j, k]) / s] + qv
        return Vector(self.m[:3, 3]), Quaternion(q), Vector((1, 1, 1))

    def __matmul__(self, o):
        if isinstance(o, Matrix):
            return Matrix(self.m @ o.m)
        raise TypeError("only Matrix @ Matrix is implemented")
