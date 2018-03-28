#
# Benedict Greenberg, March 2018
import numpy as np


class TransformFrames:
    """Stores the transformation relationship between original and target reference frames. The
    class should be instantiated with a minimum of 4 frame->frame calibration points."""
    def __init__(self, frame_a_points, frame_b_points, debug=False):
        # Check data passed into class object
        if type(frame_a_points) is not np.ndarray or type(frame_b_points) is not np.ndarray:
            raise ValueError("Calibration requires data as (n x 3) numpy array.")
        if np.shape(frame_a_points)[1] != 3 or np.shape(frame_b_points)[1] != 3:
            raise ValueError("Array should have only three dimensions.")
        if np.shape(frame_a_points)[0] < 4 or np.shape(frame_b_points)[0] < 4:
            raise ValueError("Array should have at least 4 calibration points.")
        if np.shape(frame_a_points) != np.shape(frame_b_points):
            raise ValueError("Array sizes should match.")

        self.points_from = frame_a_points
        self.points_to = frame_b_points

        number_pts = np.shape(frame_a_points)[0]
        ones = np.ones((number_pts, 1))
        mat_a = np.column_stack([frame_a_points, ones])
        mat_b = np.column_stack([frame_b_points, ones])

        self.transformation = np.linalg.lstsq(mat_a, mat_b, rcond=None)[0]
        self.transformation_reversed = np.linalg.lstsq(mat_b, mat_a, rcond=None)[0]
        if debug:
            print(self.transformation)

    def transform(self, coordinate):
        """Transforms an x,y,z coordinate to the target reference frame."""
        if np.shape(coordinate) != (3,):
            raise ValueError("Point must be a (3,) vector.")
        point = np.append(coordinate, 1)
        return np.dot(point, self.transformation)

    def transform_reversed(self, coordinate):
        """Transforms an x,y,z coordinate from the target reference frame back to the original."""
        if np.shape(coordinate) != (3,):
            raise ValueError("Point must be a (3,) vector.")
        point = np.append(coordinate, 1)
        return np.dot(point, self.transformation_reversed)


if __name__ == '__main__':
    from numpy import genfromtxt
    my_data = genfromtxt('cal_data_example.csv', delimiter=',')

    num_pts = 4

    A = my_data[0:num_pts, 0:3]
    B = my_data[0:num_pts, 3:]

    calibrate = TransformFrames(A, B)

    sample_in = my_data[0, 0:3]
    sample_out = calibrate.transform(sample_in)
    actual_out = my_data[0, 3:]
    print("Pass in coordinates: ", sample_in)
    print("Convert coordinates: ", sample_out)
    print("Correct coordinates: ", actual_out)
