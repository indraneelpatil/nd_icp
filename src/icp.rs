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
    pub fn register(&self, target_point_set: &mut PointSet<T>) -> OMatrix<f32, Dyn, Dyn> {
        let dimension = target_point_set
            .points
            .iter()
            .next()
            .expect("Input set is empty")
            .get_dimensions();

        // Initialise transformation
        let mut registration_matrix: OMatrix<f32, Dyn, Dyn> =
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
            let cross_covariance_mat = target_mat.transpose() * model_mat.clone();

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

            // Update identity matrix
            let homogenous_mat = self.get_homogeneous_matrix(&translation, &rotation, dimension);
            println!(
                " r {} test {} homo {}",
                rotation, translation, homogenous_mat
            );

            // Transform target cloud
            for point in &mut target_point_set.points {
                point.apply_transformation(&homogenous_mat);
            }

            registration_matrix *= homogenous_mat;

            // Calculate cost
            let icp_cost = self.icp_cost(&target_mat, &model_mat, &rotation);
            println!(
                "=== Finished iteration {} with cost {}",
                iteration, icp_cost
            );

            // Check termination condition
            let translation_distance = self.get_translation_distance(&translation);
            if translation_distance < self.dist_delta {
                println!(
                    "Reached termination threshold of {} with {} exiting!",
                    self.dist_delta, translation_distance
                );
                break;
            }
        }

        registration_matrix
    }
}
