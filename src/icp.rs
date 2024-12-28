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
/// 1. Profile the code
/// 2. Outlier rejection of input data
/// 3. Voxel binning
/// 4. Vectorise some stuff
/// 5. Profile the code
/// 6. Write some tests
pub struct Icp<T>
where
    T: Point + Div<f32, Output = T> + Sum<T> + Copy + Sub<T, Output = T>,
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
    T: Point + Div<f32, Output = T> + Sum<T> + Copy + Sub<T, Output = T>,
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

    pub fn get_point_correspondences(
        &self,
        target_point_set: &OMatrix<f32, Dyn, Dyn>,
        model_point_set: &OMatrix<f32, Dyn, Dyn>,
    ) -> OMatrix<f32, Dyn, Dyn> {
        let nrows = target_point_set.nrows();
        let ncols = target_point_set.ncols();

        let mut correspondence_matrix =
            OMatrix::zeros_generic(nalgebra::Dyn(nrows), nalgebra::Dyn(ncols));

        for (target_point_idx, target_point) in target_point_set.row_iter().enumerate() {
            let mut closest_model_point = None;
            let mut closest_dist = f32::MAX;
            for model_point in model_point_set.row_iter() {
                let point_diff = model_point - target_point;
                let point_dist = point_diff.norm_squared();
                if point_dist < closest_dist {
                    closest_dist = point_dist;
                    closest_model_point = Some(model_point);
                }
            }

            // Save point correspondence
            correspondence_matrix
                .row_mut(target_point_idx)
                .copy_from(&closest_model_point.expect("Failed to get closest point"));
        }
        correspondence_matrix
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

    pub fn get_homogeneous_matrix(
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

    pub fn get_translation_distance(&self, translation: &OMatrix<f32, U1, Dyn>) -> f32 {
        translation.iter().map(|val| val * val).sum::<f32>().sqrt()
    }

    pub fn icp_cost(
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

    /// ICP registrations
    ///
    /// 1. Initialise the transformation given number of dimensions
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
        let model_mat = self.get_matrix_from_point_set(&self.model_point_set.points, dimension);

        // Initialise transformation
        let mut registration_matrix: OMatrix<f32, Dyn, Dyn> =
            OMatrix::identity_generic(nalgebra::Dyn(dimension + 1), nalgebra::Dyn(dimension + 1));

        // Begin iterations
        let mut previous_cost = f32::MAX;
        for iteration in 0..self.max_iterations {
            // Find point correspondences
            let correspondence_mat = self.get_point_correspondences(&target_mat, &model_mat);

            // Remove the means from the point clouds for better rotation matrix calculation
            let mean_correspondence_point = correspondence_mat.row_mean();
            let correspondence_mat_no_mean: OMatrix<f32, Dyn, Dyn> = OMatrix::from_rows(
                &correspondence_mat
                    .row_iter()
                    .map(|row| row - mean_correspondence_point.clone_owned())
                    .collect::<Vec<_>>(),
            );

            let mean_target_point = target_mat.row_mean();
            let target_mat_no_mean: OMatrix<f32, Dyn, Dyn> = OMatrix::from_rows(
                &target_mat
                    .row_iter()
                    .map(|row| row - mean_target_point.clone_owned())
                    .collect::<Vec<_>>(),
            );

            // Calculate cross covariance
            let cross_covariance_mat =
                correspondence_mat_no_mean.transpose() * target_mat_no_mean.clone();

            // Find best rotation
            let res = nalgebra::linalg::SVD::new(cross_covariance_mat, true, true);
            let u = res.u.unwrap();
            let vt = res.v_t.unwrap();
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
            target_mat = homogenous_mat.clone() * target_mat.transpose();

            // Update registration matrix
            registration_matrix *= homogenous_mat;

            // Calculate cost
            let icp_cost = self.icp_cost(&target_mat, &model_mat, &rotation);
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
