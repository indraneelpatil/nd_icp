// Created by Indraneel on 12/8/24

use core::f32;

use nalgebra::{Const, Dyn, OMatrix, U1};

use crate::types::{Point, PointSet};

/// N dimensional Icp
///
/// # Salient features
///
/// 1. Generic Type can be any n dimensional point
/// 2. Implement Point Trait for your point
/// 3. Uses SVD to find rotation and translation
/// 3. Vectorised operations to control time complexity
///
/// # Examples
///
/// ```
///
/// let max_iterations = 100;
/// let cost_change_threshold = 1e-5;
/// let icp = Icp::new(
///    model_point_set.clone(),
///    max_iterations,
///    cost_change_threshold,
/// );
/// let result = icp.register(&target_point_set);
///
///
/// ```
///
/// # TODO:
///
/// 1. Outlier rejection of input data
/// 2. Voxel binning to make finding correspondences faster
pub struct Icp<T>
where
    T: Point + Copy,
{
    /// Model reference to register against
    model_point_set: PointSet<T>,
    /// Max number of iterations to run
    max_iterations: i32,
    /// Cost change threshold to terminate after
    cost_change_threshold: f32,
}

impl<T> Icp<T>
where
    T: Point + Copy,
{
    pub fn new(
        model_point_set: PointSet<T>,
        max_iterations: i32,
        cost_change_threshold: f32,
    ) -> Self {
        Self {
            model_point_set,
            max_iterations,
            cost_change_threshold,
        }
    }

    /// Finds closest point correspondences
    /// between two sets of points
    fn get_point_correspondences(
        &self,
        target_point_set: &OMatrix<f32, Dyn, Dyn>,
    ) -> OMatrix<f32, Dyn, Dyn> {
        let nrows = target_point_set.nrows();
        let ncols = target_point_set.ncols();

        let mut correspondence_matrix =
            OMatrix::zeros_generic(nalgebra::Dyn(nrows), nalgebra::Dyn(ncols));

        for (target_point_idx, target_point_mat) in target_point_set.row_iter().enumerate() {
            let mut closest_point: Option<T> = None;
            let mut closest_dist = f32::MAX;

            let target_point: T = Point::from_matrix(&target_point_mat);

            for model_point in self.model_point_set.points.iter() {
                let point_dist = target_point.find_distance_squared(model_point);
                if point_dist < closest_dist {
                    closest_dist = point_dist;
                    closest_point = Some(*model_point);
                }
            }

            // Save point correspondence
            correspondence_matrix
                .row_mut(target_point_idx)
                .copy_from_slice(&closest_point.expect("Closest point not found").to_vec());
        }
        correspondence_matrix
    }

    /// Converts a set of points to a Matrix with all points
    /// stacked in rows
    fn get_matrix_from_point_set(
        &self,
        point_set: &Vec<T>,
        dimension: usize,
    ) -> OMatrix<f32, Dyn, Dyn> {
        let points_vec: Vec<Vec<f32>> = point_set.iter().map(|point| point.to_vec()).collect();
        let points_vec_flattened: Vec<f32> = points_vec.into_iter().flatten().collect();
        let target_mat: OMatrix<f32, Dyn, Dyn> = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            point_set.len(),
            dimension,
            &points_vec_flattened,
        );
        target_mat
    }

    /// Converts a rotation and translation to its homogeneous matrix
    /// representation
    fn get_homogeneous_matrix(
        &self,
        translation: &OMatrix<f32, U1, Dyn>,
        rotation: &OMatrix<f32, Dyn, Dyn>,
        dimension: usize,
    ) -> OMatrix<f32, Dyn, Dyn> {
        // Start with an identity matrix
        let mut homogeneous_matrix: OMatrix<f32, Dyn, Dyn> =
            OMatrix::identity_generic(nalgebra::Dyn(dimension + 1), nalgebra::Dyn(dimension + 1));

        // Assign the rotation part
        homogeneous_matrix
            .view_mut((0, 0), (dimension, dimension))
            .copy_from(rotation);

        // Assign the translation part
        homogeneous_matrix
            .view_mut((0, dimension), (dimension, 1))
            .copy_from(&translation.transpose());

        homogeneous_matrix
    }

    /// Calculates the mean squared error between two sets of points
    fn icp_cost(
        &self,
        target_mat_no_mean: &OMatrix<f32, Dyn, Dyn>,
        model_mat_no_mean: &OMatrix<f32, Dyn, Dyn>,
        rotation: &OMatrix<f32, Dyn, Dyn>,
    ) -> f32 {
        // Rotate the target mat
        let rotated_target_mat = (rotation * target_mat_no_mean.transpose()).transpose();

        // calculate cost
        let cost = model_mat_no_mean - rotated_target_mat;
        cost.norm()
    }

    /// Subtracts the row-wise mean from a matrix and returns the resulting matrix and the mean.
    fn center_point_cloud_about_mean(
        &self,
        matrix: &OMatrix<f32, Dyn, Dyn>,
    ) -> (OMatrix<f32, Dyn, Dyn>, OMatrix<f32, Const<1>, Dyn>) {
        let mean_row = matrix.row_mean();
        let matrix_no_mean = OMatrix::from_rows(
            &matrix
                .row_iter()
                .map(|row| row - mean_row.clone_owned())
                .collect::<Vec<_>>(),
        );
        (matrix_no_mean, mean_row)
    }

    /// Applies a homogenous transformation to a matrix of points
    fn transform_matrix(
        &self,
        matrix: &mut OMatrix<f32, Dyn, Dyn>,
        homogeneous_transformation_matrix: &OMatrix<f32, Dyn, Dyn>,
    ) {
        let nrows = matrix.nrows();
        let ncols = matrix.ncols();

        let mut homogeneous_representation = matrix.clone_owned();
        homogeneous_representation = homogeneous_representation.insert_column(ncols, 1.0);

        // Apply the homogeneous transformation
        let transformed_homogeneous_matrix =
            homogeneous_transformation_matrix * homogeneous_representation.transpose();

        *matrix = transformed_homogeneous_matrix
            .transpose()
            .view((0, 0), (nrows, ncols))
            .into_owned();
    }

    /// ICP registration
    ///
    /// 1. Initialises the transformation given number of dimensions
    /// 2. For each point
    ///     - Find closest point in reference cloud
    /// 3. Remove the means
    /// 4. Find transformaton and rotation which will minimise the error
    /// 5. Transform the target cloud
    /// 6. Iterate until within error threshold or max iterations
    pub fn register(&self, target_point_set: &PointSet<T>) -> OMatrix<f32, Dyn, Dyn> {
        let dimension = target_point_set
            .points
            .iter()
            .next()
            .expect("Input set is empty")
            .get_dimensions();

        // Vectorise point sets
        let mut target_mat = self.get_matrix_from_point_set(&target_point_set.points, dimension);

        // Initialise transformation
        let mut registration_matrix: OMatrix<f32, Dyn, Dyn> =
            OMatrix::identity_generic(nalgebra::Dyn(dimension + 1), nalgebra::Dyn(dimension + 1));

        // Begin iterations
        let mut previous_cost = f32::MAX;
        for iteration in 0..self.max_iterations {
            // Find point correspondences
            let correspondence_mat = self.get_point_correspondences(&target_mat);

            // Remove the means from the point clouds for better rotation matrix calculation
            let (correspondence_mat_no_mean, mean_correspondence_point) =
                self.center_point_cloud_about_mean(&correspondence_mat);
            let (target_mat_no_mean, mean_target_point) =
                self.center_point_cloud_about_mean(&target_mat);

            // Calculate cross covariance
            let cross_covariance_mat =
                correspondence_mat_no_mean.transpose() * target_mat_no_mean.clone();

            // Find best rotation
            let res = nalgebra::linalg::SVD::new(cross_covariance_mat, true, true);
            let u = res.u.expect("Failed to calculate u matrix");
            let vt = res.v_t.expect("Failed to calculate vt matrix");
            let rotation = u * vt;

            // Find translation
            let translation = mean_correspondence_point
                - (rotation.clone() * mean_target_point.transpose()).transpose();

            let homogenous_mat = self.get_homogeneous_matrix(&translation, &rotation, dimension);
            println!(
                " r {} test {} homo {}",
                rotation, translation, homogenous_mat
            );

            // Transform target cloud
            self.transform_matrix(&mut target_mat, &homogenous_mat);

            // Update registration matrix
            registration_matrix *= homogenous_mat;

            // Calculate cost
            let icp_cost =
                self.icp_cost(&target_mat_no_mean, &correspondence_mat_no_mean, &rotation);
            println!(
                "=== Finished iteration {} with cost {}",
                iteration, icp_cost
            );

            // Check termination condition
            if (previous_cost - icp_cost).abs() < self.cost_change_threshold {
                println!(
                    "Reached termination threshold of {} with {} exiting!",
                    self.cost_change_threshold, previous_cost
                );
                break;
            }
            previous_cost = icp_cost;
        }

        registration_matrix
    }
}

#[cfg(test)]
mod tests {
    use crate::types::Point3D;

    use super::*;

    use rstest::*;

    #[fixture]
    fn icp_fixture() -> Icp<Point3D> {
        let max_iterations = 1;
        let cost_change_threshold = 1e-3;

        let model_point_set = PointSet {
            points: vec![
                Point3D {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                Point3D {
                    x: 2.0,
                    y: 2.0,
                    z: 2.0,
                },
                Point3D {
                    x: 3.0,
                    y: 3.0,
                    z: 3.0,
                },
            ],
        };

        Icp::new(model_point_set, max_iterations, cost_change_threshold)
    }

    #[rstest]
    fn test_get_matrix_from_point_set(icp_fixture: Icp<Point3D>) {
        let expected_matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0],
        );

        let result_matrix =
            icp_fixture.get_matrix_from_point_set(&icp_fixture.model_point_set.points, 3);

        assert_eq!(result_matrix, expected_matrix);
        assert_eq!(result_matrix.row(0), expected_matrix.row(0));
        assert_eq!(result_matrix.row(1), expected_matrix.row(1));
        assert_eq!(result_matrix.row(2), expected_matrix.row(2));
    }

    #[rstest]
    fn test_get_point_correspondences(icp_fixture: Icp<Point3D>) {
        let target_matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0],
        );

        let correspondence_mat = icp_fixture.get_point_correspondences(&target_matrix);

        assert_eq!(correspondence_mat, target_matrix);

        // Shuffled order
        let target_shuffled = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[3.0, 3.0, 3.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0],
        );

        let correspondence_mat = icp_fixture.get_point_correspondences(&target_shuffled);

        assert_eq!(correspondence_mat, target_shuffled);
    }

    #[rstest]
    fn test_center_point_cloud_about_mean(icp_fixture: Icp<Point3D>) {
        let model_matrix =
            icp_fixture.get_matrix_from_point_set(&icp_fixture.model_point_set.points, 3);

        let expected_mean = OMatrix::<f32, Dyn, Dyn>::from_row_slice(1, 3, &[2.0, 2.0, 2.0]);

        let expected_matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[
                -1.0, -1.0, -1.0, // First row - mean
                0.0, 0.0, 0.0, // Second row - mean
                1.0, 1.0, 1.0, // Third row - mean
            ],
        );

        // Call the function
        let (result_matrix, result_mean) = icp_fixture.center_point_cloud_about_mean(&model_matrix);

        assert_eq!(result_matrix, expected_matrix);
        assert_eq!(result_mean, expected_mean);

        let another_matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
        );

        let expected_mean = OMatrix::<f32, Dyn, Dyn>::from_row_slice(1, 3, &[4.0, 5.0, 6.0]);

        let expected_matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[-3.0, -3.0, -3.0, 0.0, 0.0, 0.0, 3.0, 3.0, 3.0],
        );

        // Call the function
        let (result_matrix, result_mean) =
            icp_fixture.center_point_cloud_about_mean(&another_matrix);

        assert_eq!(result_matrix, expected_matrix);
        assert_eq!(result_mean, expected_mean);
    }

    #[rstest]
    fn test_get_homogeneous_matrix(icp_fixture: Icp<Point3D>) {
        // Sample rotation matrix (identity for simplicity)
        let rotation = OMatrix::<f32, Dyn, Dyn>::identity_generic(Dyn(3), Dyn(3));

        // Sample translation vector
        let translation = OMatrix::<f32, U1, Dyn>::from_row_slice(&[1.0, 2.0, 3.0]);

        // Expected homogeneous matrix
        let expected_matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            4,
            4,
            &[
                1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0,
            ],
        );

        let result_matrix = icp_fixture.get_homogeneous_matrix(&translation, &rotation, 3);

        assert_eq!(result_matrix, expected_matrix);
    }

    #[rstest]
    fn test_transform_matrix(icp_fixture: Icp<Point3D>) {
        let mut matrix = OMatrix::<f32, Dyn, Dyn>::from_row_slice(2, 2, &[1.0, 2.0, 3.0, 4.0]);
        let transformation = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        );

        let expected = OMatrix::<f32, Dyn, Dyn>::from_row_slice(2, 2, &[1.0, 2.0, 3.0, 4.0]);

        icp_fixture.transform_matrix(&mut matrix, &transformation);
        assert_eq!(matrix, expected);

        // Translation
        let transformation = OMatrix::<f32, Dyn, Dyn>::from_row_slice(
            3,
            3,
            &[1.0, 0.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0],
        );

        let expected = OMatrix::<f32, Dyn, Dyn>::from_row_slice(2, 2, &[2.0, 4.0, 4.0, 6.0]);

        icp_fixture.transform_matrix(&mut matrix, &transformation);
        assert_eq!(matrix, expected);
    }

    #[rstest]
    fn test_icp_registration(icp_fixture: Icp<Point3D>) {
        let target_point_set = PointSet {
            points: vec![
                Point3D {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
                Point3D {
                    x: 2.0,
                    y: 2.0,
                    z: 2.0,
                },
                Point3D {
                    x: 3.0,
                    y: 3.0,
                    z: 3.0,
                },
                Point3D {
                    x: 3.0,
                    y: 3.0,
                    z: 3.0,
                },
            ],
        };

        let result = icp_fixture.register(&target_point_set);

        assert!(result[(0, 3)].abs() < 1e-3);
        assert!(result[(1, 3)].abs() < 1e-3);
        assert!(result[(2, 3)].abs() < 1e-3);
    }
}
