"""Regression checks for clean CAD material seams and continuous shading."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "colorize_ur7e", ROOT / "src/arm_description/scripts/colorize_ur7e.py",
)
COLOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COLOR)


def area(triangles):
    return np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1) / 2


class VisualMaterialsTest(unittest.TestCase):
    def test_long_facet_is_split_at_color_boundary_without_changing_surface(self):
        source = np.array([[[-0.4, 0, 0.14], [-0.2, 0, 0.14], [-0.2, 0.04, 0.14]]])
        normals = np.broadcast_to([0., 0., 1.], source.shape).copy()
        triangles, normals = COLOR.split_material_boundaries("link_2", source, normals)
        self.assertGreater(len(triangles), 1)
        self.assertAlmostEqual(area(source).sum(), area(triangles).sum(), places=14)
        self.assertTrue(np.all(area(triangles) > 0))
        np.testing.assert_allclose(triangles[:, :, 2], 0.14)
        np.testing.assert_allclose(normals[:, :, 2], 1)
        self.assertEqual(set(COLOR.surface_materials("link_2", triangles)), {"graphite", "silver"})
        for axis, cut in COLOR.MATERIAL_PLANES["link_2"]:
            crossing = ((triangles[:, :, axis].min(1) < cut - 1e-12)
                        & (triangles[:, :, axis].max(1) > cut + 1e-12))
            self.assertFalse(crossing.any())

    def test_cut_interpolates_original_normals_at_shared_vertices(self):
        triangles = np.array([[[0., 0., 0.], [2., 0., 0.], [0., 2., 0.]]])
        normals = np.array([[[0., 0., 1.], [1., 0., 1.], [0., 1., 1.]]])
        pieces, shading = COLOR.split_at_plane(triangles, normals, 0, 1.)
        cut_corner = np.all(np.isclose(pieces, [1., 0., 0.]), axis=2)
        self.assertGreater(cut_corner.sum(), 1)
        np.testing.assert_allclose(shading[cut_corner], np.tile([0.5, 0., 1.], (cut_corner.sum(), 1)))
        self.assertAlmostEqual(area(triangles).sum(), area(pieces).sum())

    def test_cut_through_vertex_preserves_winding_and_area(self):
        for sign in (-1, 1):
            triangle = np.array([[[0., 1., 0.], [1., 0., 0.], [-1., 0., 0.]]])
            if sign == -1:
                triangle = triangle[:, ::-1]
            normal = np.cross(triangle[:, 1] - triangle[:, 0], triangle[:, 2] - triangle[:, 0])
            normals = np.repeat(normal[:, None], 3, axis=1)
            pieces, _ = COLOR.split_at_plane(triangle, normals, 0, 0.)
            face = np.cross(pieces[:, 1] - pieces[:, 0], pieces[:, 2] - pieces[:, 0])
            self.assertTrue(np.all(face @ normal[0] > 0))
            self.assertEqual(len(pieces), 2)
            self.assertAlmostEqual(area(triangle).sum(), area(pieces).sum())

    def test_exported_meshes_have_no_facets_crossing_material_boundaries(self):
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        for link, planes in COLOR.MATERIAL_PLANES.items():
            with self.subTest(link=link):
                root = ET.parse(COLOR.LOCAL_MESHES / f"{link}.dae").getroot()
                mesh = root.find(".//c:mesh", ns)
                arrays = {a.get("id"): np.fromstring(a.text, sep=" ").reshape(-1, 3)
                          for a in mesh.findall("c:source/c:float_array", ns)}
                normals = arrays["normals-array"]
                self.assertTrue(np.isfinite(normals).all())
                np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1., atol=2e-6)
                for group in mesh.findall("c:triangles", ns):
                    indices = np.fromstring(group.find("c:p", ns).text, sep=" ", dtype=int).reshape(-1, 3, 2)
                    triangles = arrays["positions-array"][indices[:, :, 0]]
                    for axis, cut in planes:
                        crossing = ((triangles[:, :, axis].min(1) < cut - 1e-8)
                                    & (triangles[:, :, axis].max(1) > cut + 1e-8))
                        self.assertFalse(crossing.any(), (link, group.get("material"), axis, cut))


if __name__ == "__main__":
    unittest.main()
