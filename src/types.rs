// Created by Indraneel on 7th Dec
use nalgebra::{Vector3, Vector4};

pub struct PointSet<T> {
    pub points: Vec<T>,
}

pub struct Point3D {
    /// Translation XYZ
    pub t: Vector3<f32>,
    /// Quaternion Rotation
    pub r: Vector4<f32>,
}
