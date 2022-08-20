import cv2
import numpy as np
from scipy.ndimage import map_coordinates


class PanoramicImageTool(object):
    # TODO: make method names be consistent (2018/10/21, SG)
    def __init__(self, panoramic_image=None):
        self.panoramic_image = panoramic_image
        if panoramic_image:
            self.height, self.width = self.panoramic_image.shape[:-1]
        self.azimuth_origin = 0  # heading [-pi, pi]
        self.altitude_origin = 0  # pitch [-pi/2, pi/2]
        self.fov = np.pi / 2.
        self.panoramic_image_shape = (4096, 8192, 3)
        self._face_to_index = {'left': 0, 'right': 1, 'top': 2, 'bottom': 3, 'front': 4, 'back': 5}
        self.sphere_coords = self.make_spherical_coordinates(self.panoramic_image_shape[:-1])
        self.face_index_map = self.get_face_index(self.sphere_coords)
        self.raw_coords = self.set_raw_coords(self.face_index_map, self.sphere_coords, self.panoramic_image_shape)
        self.cube_coords = self.raw_to_cube_coords(self.raw_coords, self.panoramic_image_shape[0])

    def set_panoramic_image(self, image):
        self.panoramic_image = image.astype(np.float32)
        self.height, self.width = image.shape[:-1]

    def set_view_angles(self, heading, pitch, fov=90):
        self.set_azimuth(heading)
        self.set_altitude(pitch)
        self.set_fov(fov)

    def set_azimuth(self, heading):
        self.azimuth_origin = np.deg2rad(heading)

    def set_altitude(self, pitch):
        self.altitude_origin = np.deg2rad(pitch)

    def set_fov(self, fov):
        self.fov = np.deg2rad(fov)

    def crop(self, target_shape):
        """
        refer to J. Xiao et al., Recongnizing Scene Viewpoint using Panoramic Place Representation, CVPR2012
        """
        target_height, target_width = target_shape
        if target_width > target_height:
            crop_px = target_width
        else:
            crop_px = target_height
        px, py = self.make_texture_map_for_cylindrical(crop_px)
        panoramic_image_augmented_bounds = self.augment_boundaries()
        cropped_image = self.warp_image_fast(panoramic_image_augmented_bounds, px, py)
        cropped_image = self.rescale(cropped_image, (target_height, target_width))
        return cropped_image.astype(np.uint8)

    def make_texture_map_for_cylindrical(self, width):
        ang_x, ang_y = self.set_angles_look_at(width)
        px, py = self.get_mapping_funcs_from_angles(ang_x, ang_y)
        px = px.reshape(width, width)
        py = py.reshape(width, width)
        return px, py

    def set_angles_look_at(self, width):
        def trim_horizontal(ang_x):
            ind_x = np.nonzero(ang_x <= -np.pi)
            ang_x[ind_x] = ang_x[ind_x] + 2. * np.pi
            ind_x = np.nonzero(ang_x > np.pi)
            ang_x[ind_x] = ang_x[ind_x] - 2. * np.pi
            return ang_x

        def trim_vertical(ang_x, ang_y):
            ind_y = np.nonzero(ang_y < -np.pi / 2)
            ang_y[ind_y] = -np.pi - ang_y[ind_y]
            ang_x[ind_y] = ang_x[ind_y] + np.pi
            return ang_x, ang_y

        x, y, z, ind_valid = self.set_3d_coord_grids(width)
        ang_x, ang_y = self.project_to_sphere(x, y, z, ind_valid)
        ang_x, ang_y = trim_vertical(ang_x, ang_y)
        ang_x = trim_horizontal(ang_x)
        return ang_x, ang_y

    def set_3d_coord_grids(self, width):
        def make_mesh_grids(width):
            tx, ty = np.meshgrid(range(width), range(width))
            tx = tx - 0.5 - width / 2.
            ty = ty - 0.5 - width / 2.
            return tx, ty

        tx, ty = make_mesh_grids(width)
        r = (width / 2.) / np.tan(self.fov / 2.)

        radius = np.sqrt(ty ** 2 + r ** 2)
        ang_y = np.arctan(-ty / r)
        ang_y = ang_y + self.altitude_origin
        ind_valid = np.nonzero(np.fabs(ang_y) > np.pi / 2)

        x = np.sin(ang_y) * radius
        y = -np.cos(ang_y) * radius
        z = tx
        return x, y, z, ind_valid

    def project_to_sphere(self, x, y, z, ind_n):
        ang_x = np.arctan(-z / y)
        rzy = np.sqrt(z ** 2 + y ** 2)
        ang_y = np.arctan(x / rzy)
        ang_x[ind_n] = ang_x[ind_n] + np.pi
        ang_x = ang_x + self.azimuth_origin
        return ang_x, ang_y

    def get_mapping_funcs_from_angles(self, ang_x, ang_y):
        px = (ang_x + np.pi) / (2. * np.pi) * self.width + 0.5
        py = (-ang_y + np.pi / 2.) / np.pi * self.height + 0.5
        ind_x2 = np.nonzero(px < 1)
        px[ind_x2] = px[ind_x2] + self.width
        return px, py

    def augment_boundaries(self, margin=1):
        """
        to make seamless output around hfov 180
        """
        img = np.empty(shape=(self.height, self.width + margin, 3))
        img[:, :self.width, :] = self.panoramic_image
        img[:, -margin:, :] = self.panoramic_image[:, :margin, :]
        return img

    @staticmethod
    def warp_image_fast(img, xx, yy):
        minx, miny, maxx, maxy = PanoramicImageTool.get_boundaries(img, xx, yy)
        img = img[miny:maxy, minx:maxx, :]  # values
        points = np.vstack((yy.flatten() - miny, xx.flatten() - minx))
        h, w = xx.shape
        warped_image = np.empty(shape=(h, w, 3))
        for c in range(3):
            # spline ordering has trade-off relation between quality vs. speed
            colors = map_coordinates(img[:, :, c], points, order=1, mode='wrap')
            warped_image[:, :, c] = colors.reshape(h, w)
        return warped_image

    @staticmethod
    def get_boundaries(img, xx, yy):
        def get_min_value(x, lower=0):
            return int(max(lower, np.floor(np.min(x))))

        def get_max_value(x, upper):
            return int(min(upper, np.ceil(np.max(x))))

        height, width = img.shape[:-1]
        minx = get_min_value(xx)
        miny = get_min_value(yy)
        maxx = get_max_value(xx, width-1)
        maxy = get_max_value(yy, height-1)
        return minx, miny, maxx, maxy

    def rescale(self, img, shape):
        height, width = shape
        # img = img / 255
        if width > height:
            xrange = np.array(range(width)).astype(np.int)
            yrange = (np.array(range(height)) + (width-height)/2).astype(np.int)
        else:
            xrange = (np.array(range(width)) + (height - width) / 2).astype(np.int)
            yrange = np.array(range(height)).astype(np.int)
        img = img[:, xrange, :]
        img = img[yrange, :, :]
        return img

    # TODO: cubic_to_panorama isn't fast... need profiling to enhance the exec time (2018/10/21, SG)
    def cubic_to_panorama(self, cubic_images):
        """
        refer to https://en.wikipedia.org/wiki/Cube_mapping
        """
        height = cubic_images['top'].shape[0]
        for face, img in cubic_images.items():
            cubic_images[face] = cv2.resize(img, (height*2, height*2))
        height *= 2
        width = 2 * height
        panoramic_image_shape = (height, width, 3)

        color_map = self.get_color_map_all_faces(cubic_images, panoramic_image_shape[0])
        panoramic_image = self.set_panoramic_image_texture_using_index_maps(
                color_map, self.cube_coords, self.face_index_map, panoramic_image_shape)
        return panoramic_image

    def get_color_map_all_faces(self, cubic_images, height):
        color_map = np.zeros((3, height, height, 6), np.uint8)
        for face, img in cubic_images.items():
            index = self._face_to_index[face]
            color_map[..., index] = np.transpose(img, (2, 0, 1))
        return color_map

    def make_spherical_coordinates(self, shape):
        height, width = shape
        tx, ty = np.meshgrid(range(width), range(height))
        ty = 2 * ty / height - 1
        tx = 2 * tx / width - 1
        theta = np.pi * tx
        phi = (np.pi / 2) * ty

        x = np.cos(phi) * np.cos(theta)
        y = np.sin(phi)
        z = np.cos(phi) * np.sin(theta)
        return np.dstack((x, y, z))

    def get_face_index(self, sphere_coords):
        def set_negative_face_index(face_index_map, frontal_face_coords, coords, face_name):
            ind = np.bitwise_and(np.fabs(frontal_face_coords - coords) < 0.00001, coords < 0)
            face_index_map[ind] = self._face_to_index[face_name]
            return face_index_map

        def set_positive_face_index(face_index_map, frontal_face_coords, coords, face_name):
            ind = np.bitwise_and(np.fabs(frontal_face_coords - coords) < 0.00001, coords >= 0)
            face_index_map[ind] = self._face_to_index[face_name]
            return face_index_map

        max_indices = np.argmax(np.fabs(sphere_coords), axis=2)
        frontal_face_coords = np.zeros(max_indices.shape)
        for i in range(3):
            frontal_face_coords[max_indices == i] = sphere_coords[..., i][max_indices == i]

        face_index_map = -1 * np.ones(frontal_face_coords.shape, dtype=np.int)
        face_index_map = set_negative_face_index(face_index_map, frontal_face_coords, sphere_coords[..., 0], 'back')
        face_index_map = set_positive_face_index(face_index_map, frontal_face_coords, sphere_coords[..., 0], 'front')
        face_index_map = set_negative_face_index(face_index_map, frontal_face_coords, sphere_coords[..., 1], 'top')
        face_index_map = set_positive_face_index(face_index_map, frontal_face_coords, sphere_coords[..., 1], 'bottom')
        face_index_map = set_negative_face_index(face_index_map, frontal_face_coords, sphere_coords[..., 2], 'left')
        face_index_map = set_positive_face_index(face_index_map, frontal_face_coords, sphere_coords[..., 2], 'right')

        return face_index_map

    def set_raw_coords(self, face_index_map, sphere_coords, shape):
        raw_coords = -1 * np.ones(shape)

        ind = (face_index_map == self._face_to_index['back'])
        raw_coords[..., 0][ind] = -sphere_coords[..., 2][ind]
        raw_coords[..., 1][ind] = sphere_coords[..., 1][ind]
        raw_coords[..., 2][ind] = sphere_coords[..., 0][ind]

        ind = (face_index_map == self._face_to_index['front'])
        raw_coords[..., 0][ind] = sphere_coords[..., 2][ind]
        raw_coords[..., 1][ind] = sphere_coords[..., 1][ind]
        raw_coords[..., 2][ind] = sphere_coords[..., 0][ind]

        ind = (face_index_map == self._face_to_index['top'])
        raw_coords[..., 0][ind] = sphere_coords[..., 2][ind]
        raw_coords[..., 1][ind] = sphere_coords[..., 0][ind]
        raw_coords[..., 2][ind] = sphere_coords[..., 1][ind]

        ind = (face_index_map == self._face_to_index['bottom'])
        raw_coords[..., 0][ind] = sphere_coords[..., 2][ind]
        raw_coords[..., 1][ind] = -sphere_coords[..., 0][ind]
        raw_coords[..., 2][ind] = sphere_coords[..., 1][ind]

        ind = (face_index_map == self._face_to_index['left'])
        raw_coords[..., 0][ind] = sphere_coords[..., 0][ind]
        raw_coords[..., 1][ind] = sphere_coords[..., 1][ind]
        raw_coords[..., 2][ind] = sphere_coords[..., 2][ind]

        ind = (face_index_map == self._face_to_index['right'])
        raw_coords[..., 0][ind] = -sphere_coords[..., 0][ind]
        raw_coords[..., 1][ind] = sphere_coords[..., 1][ind]
        raw_coords[..., 2][ind] = sphere_coords[..., 2][ind]

        return raw_coords

    def raw_to_cube_coords(self, raw_coords, height):
        coords_x = ((raw_coords[..., 0]/np.fabs(raw_coords[..., 2])) + 1.)/2.
        coords_y = ((raw_coords[..., 1]/np.fabs(raw_coords[..., 2])) + 1.)/2.
        cube_coords = np.dstack((coords_x, coords_y))
        cube_coords = np.round(cube_coords * height)
        cube_coords = np.clip(cube_coords, 0, height-1)
        return cube_coords

    def set_panoramic_image_texture_using_index_maps(self, color_map, cube_coords, face_index_map, shape):
        height, width, _ = shape
        panoramic_image = np.zeros(shape=shape, dtype=np.uint8)
        sub2ind = cube_coords[..., 1] + cube_coords[..., 0]*height + face_index_map*height*height
        sub2ind = sub2ind.astype(np.int)
        for i in range(3):
            panoramic_image[..., i] = color_map[i, ...].ravel(order='F')[sub2ind]
        return panoramic_image


# unit test
if __name__ == '__main__':

    import argparse
    parser = argparse.ArgumentParser("Crop image")
    parser.add_argument("--image", type=str, default="test.jpg", help='target image')
    parser.add_argument("--tilt", type=float, default=0.0, help='tilt angle [deg]')
    parser.add_argument("--pan", type=float, default=0.0, help='pan angle [deg]')
    parser.add_argument("--fov", type=float, default=70.0, help='fov [deg]')
    parser.add_argument("--width", type=int, default=1280, help='width [px]')
    parser.add_argument("--height", type=int, default=960, help='height [px]')
    args = parser.parse_args()

    pano_tool = PanoramicImageTool()
    pano_image = cv2.imread(args.image)
    pano_tool.set_panoramic_image(pano_image)

    pano_tool.set_view_angles(args.pan, args.tilt, args.fov)

    crop_image = pano_tool.crop((args.height, args.width))
    cv2.imshow('cropped_img', crop_image)
    cv2.imshow("pano", pano_image)
    cv2.waitKey(0)

