// Data structure to hold a point
struct PointXYZ {
    x : f32,
    y : f32,
    z : f32,
};


// Data structure to hold a set of points
struct PointCloud {
    point_vec: Vec<PointXYZ>,
};


// Translational transform
struct Translate3D {
    x: f32,
    y: f32,
    z: f32,
};

impl Translate3D {

    fn default() -> Self {
    
    
    }
}

/*
    ICP Algorithm
    
    1.  Initialise number of iterations and threshold error to exit iterations, magnitude of step
    2.  Loop over iterations, in each iteration calculate the error between corresponding
       points
    3. Take average error in all points and then calculate step in that direction
    4. Step all points in that direction, keep a track of the summation of error
    5. Repeat until threshold is met
*/


fn euclidean_distance(point1: PointXYZ, point2: PointXYZ) -> f32 {

    sqrt(pow((point1.x-point2.x),2)+ pow((point1.y-point2.y),2)+ pow((point1.z-point2.z),2))
}

fn closest_point(point: PointXYZ, cloud: PointCloud) -> i32 {

   let mut closest_distance = f32::max();
   let closest_index = -1;
   
   for i:cloud.point_vec.size() {
   
       let dist = euclidean_distance(point, cloud.point_vec[i]);
       if dist < closest_distance {
           closest_distance = dist;
           closest_index = i;
       }
   }
   
   return closest_index;
}

fn calculate_error(cloud1: PointCloud, cloud2: PointCloud) -> Translate3D  {

    let size_cloud_1 = cloud1.point_vec.size();
    let size_cloud_2 = cloud2.point_vec.size();
    
    let num_points = min(size_cloud_1,size_cloud_2);
    let mut error = Translate3D::default();
    
    for(auto& point_idx:num_points) {
    
        let point1 = cloud1.point_vec[point_idx];
        
         // find the closest point
         let closest_point_idx = closest_point(point1, cloud2);
        
        let point2 = cloud2.point_vec[closest_point_idx];
        
        
        error.x += (point2.x - point1.x);
        error.y += (point2.y - point1.y);
        error.z += (point2.z - point1.z);
    }
      
    // Return average error
    error.x /= num_points;
    error.y /= num_points;
    error.z /= num_points;
      
    return error;
}

fn transform_cloud(cloud1: PointCloud, transform:Translate3D ) -> PointCloud {
    
}

fn Icp(cloud1: PointCloud, cloud2: PointCloud) -> Translate3D {

    let num_iterations = 1000;
    let error_threhold_to_exit_m = 0.01;
    let step_magnitude_per_cent = 0.01;
    
    let mut icp_transform = Translate3D {
           x: 0.0,
           y: 0.0,
           z: 0.0,
    };
 
    for(auto& i:num_iterations) {
    
             Translate3D error = calculate_error(cloud1, cloud2);
         
             // Save error
              icp_transform.x += error.x;
              icp_transform.y += error.y;
              icp_transform.z += error.z;
              
             // Check if error within threshold
             if fabs(error.x <= error_threhold_to_exit_m) && fabs(error.x <= error_threhold_to_exit_m) && fabs(error.x <= error_threhold_to_exit_m) {
             
             break;
             }
             
             
             cloud1 = transform_cloud(cloud1, error);
             
             }
        
        return icp_transform;  
    }
