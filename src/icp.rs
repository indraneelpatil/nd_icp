// Created by Indraneel on 12/8/24

use core::f32;
use std::{
    iter::Sum,
    ops::{Div, Sub},
};

use nalgebra::{Dyn, OMatrix, U1, U4};

use crate::types::{Point, PointSet};

/// N dimensional Icp
///
/// Type can be a any n dimensional point
/// TODO:
/// 1. Outlier rejection of input data
/// 2. Voxel binning
pub struct Icp<T>
where
    T: Point + Div<f32, Output = T> + Sum<T> + Copy + Sub<T, Output = T>,
{
    /// Model reference to register against
    model_point_set: PointSet<T>,
    /// Max number of iterations to run
    max_iterations: i32,
    /// Distance threshold to terminate after
    dist_delta: f32,
}

impl<T> Icp<T>
where
    T: Point + Div<f32, Output = T> + Sum<T> + Copy + Sub<T, Output = T>,
{
    pub fn new(model_point_set: PointSet<T>, max_iterations: i32, dist_delta: f32) -> Self {
        Self {
            model_point_set,
            max_iterations,
            dist_delta,
        }
    }

    pub fn get_point_correspondences(&self, target_point_set: &PointSet<T>) -> Vec<T> {
        let mut model_point_correspondences = vec![];
        for target_point in &target_point_set.points {
            let mut closest_point_idx = None;
            let mut closest_dist = f32::MAX;
            for (model_point_idx, model_point) in self.model_point_set.points.iter().enumerate() {
                let point_dist = target_point.find_distance_squared(model_point);
                if point_dist < closest_dist {
                    closest_dist = point_dist;
                    closest_point_idx = Some(model_point_idx);
                }
            }

            // Save point correspondence
            model_point_correspondences.push(
                self.model_point_set
                    .points
                    .get(closest_point_idx.expect("Failed to get closest point"))
                    .copied()
                    .expect("Invalid closest point"),
            );
        }
        model_point_correspondences
    }

    pub fn get_matrix_from_point_set(
        &self,
        point_set: &Vec<T>,
        dimension: usize,
    ) -> OMatrix<f32, Dyn, Dyn> {
        let points_vec: Vec<Vec<f32>> = point_set.iter().map(|point| point.to_vec()).collect();
        let points_vec_flattened: Vec<f32> = points_vec.into_iter().flatten().collect();
        let target_mat: OMatrix<f32, Dyn, Dyn> =
            OMatrix::from_vec_generic(Dyn(point_set.len()), Dyn(dimension), points_vec_flattened);
        target_mat
    }

    pub fn get_homogenous_matrix(
        &self,
        translation: &OMatrix<f32, U1, Dyn>,
        rotation: &OMatrix<f32, Dyn, Dyn>,
    ) -> OMatrix<f32, Dyn, Dyn> {
        unimplemented!()
    }

    /// ICP registrations
    ///
    /// 1. Initialise the transformation given number of dimensions
    /// 2. For each point
    ///     - Find closest point in reference cloud
    /// 3. Remove the means
    /// 4. Find transformaton and rotation which will minimise the error
    /// 5. Transform the target cloud
    /// 6. Iterate until within error threshold or max iterations
    pub fn register(&self, target_point_set: PointSet<T>) -> OMatrix<f32, Dyn, Dyn> {
        let dimension = target_point_set
            .points
            .iter()
            .next()
            .expect("Input set is empty")
            .get_dimensions();

        // Initialise transformation
        let mut identity_matrix: OMatrix<f32, Dyn, Dyn> =
            OMatrix::identity_generic(nalgebra::Dyn(dimension + 1), nalgebra::Dyn(dimension + 1));

        // Begin iterations
        for iteration in 0..self.max_iterations {
            // Find point correspondences
            let model_point_correspondences = self.get_point_correspondences(&target_point_set);

            // Remove the means from the point clouds
            let model_set_mean: T = model_point_correspondences
                .iter()
                .map(|point| *point / model_point_correspondences.len() as f32)
                .sum();
            let model_point_correspondences_no_mean: Vec<T> = model_point_correspondences
                .iter()
                .map(|model_point| *model_point - model_set_mean)
                .collect();
            let target_set_mean: T = target_point_set
                .points
                .iter()
                .map(|point| *point / target_point_set.points.len() as f32)
                .sum();
            let target_points_no_mean: Vec<T> = target_point_set
                .points
                .iter()
                .map(|target_point| *target_point - target_set_mean)
                .collect();

            // Calculate cross covariance
            let target_mat = self.get_matrix_from_point_set(&target_points_no_mean, dimension);
            let model_mat =
                self.get_matrix_from_point_set(&model_point_correspondences_no_mean, dimension);
            let cross_covariance_mat = target_mat.transpose() * model_mat;

            // Find best rotation
            let res = nalgebra::linalg::SVD::new(cross_covariance_mat, true, true);
            let u = res.u.unwrap();
            let vt = res.v_t.unwrap();
            let rotation = u * vt;

            // Find translation
            let target_set_mean_mat =
                OMatrix::from_vec_generic(U1, Dyn(dimension), target_set_mean.to_vec());
            let model_set_mean_mat =
                OMatrix::from_vec_generic(U1, Dyn(dimension), model_set_mean.to_vec());
            let translation = target_set_mean_mat
                - (rotation.clone() * model_set_mean_mat.transpose()).transpose();

            println!(" r {} test {}", rotation, translation);

            // Update identity matrix
            let homogenous_mat = self.get_homogenous_matrix(&translation, &rotation);
            identity_matrix *= homogenous_mat;

            // Transform target cloud

            // Check termination condition

            // Calculate cost

            println!("=== Finished iteration {} with cost {}", iteration, 0.0);
        }

        identity_matrix
    }
}
