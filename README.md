### Preliminaries
#### Matrices
* Is a function which takes in an input and gives an output
* Usually applies a "transformation" to the input
* Can also be a composition of sequential transformations

#### Projection transformation
* A matrix which when multiplied to any vector projects that vector to its closest point on a particular subspace
* By virtue it also compresses the vector to a lower dimension

#### Matrix inverse
* The matrix which applies the opposite transformation to the original matrix
* Unfortunately this isnt unique

#### Eigenvectors/ Eigenvalues of matrix
* Vector which only gets scaled after matrix transformation but maintains direction
* Eigen value is the scaling constant for the eigen vector
* Intuitively they point in the direction of most significant variation

#### Diagonal matrices
* Diagonal matrices scale a vector uniformly in all directions without rotating or changing direction of the vector
* Special case: Identity matrix which scales everything by 1

#### Rectangular matrices
* Rectangular matrices can add or subtract a dimension when applied to a vector

#### Symmetric matrices
* Special property that eigen vectors are orthogonal
* So if we take eigen vectors and put them into a matrix then transpose of that transforms eigen vectors to align with standard basis
* If we dont take transpose that matrix transforms standard basis to eigen vectors

#### AA^T
* This gives us a symmetric matrix from a non symmetric matrix

#### Singular value decomposition
* AA^T is the left symmetrix matrix S_L and A^TA is the right symmetrix matrix S_R
* Eigen vectors of S_L are called left singular vectors and a matrix of those eigen vectors is called the left singular matrix
* Eigen vectors of S_R are called right singular vectors and a matrix of those eigen vectors is called right singular matrix
* Square root of those eigen values of the above eigen vectors are called singular values of the original matrix A
* Singular value decomposition: Any matrix can be represented as a matrix multiplication of three special matrices where 1st and 3rd are orthonagonal matrices (composed of the singular vectors) which cause rotation but second matrix is diagonal (composed of the singular values) and causes stretching
* Intuitively we are rotating the eigen vectors to align with standard basis and then the diagonal rectangular matrix is the dimension eraser which is removing a dimension (not always sometimes it just causes stretching, depends on if the original matrix is reactangular or not) and then we are rotating it back to align with the eigen vectors
* Goal of SVD is to decompose a transformation into the above three sequential actions

#### Questions
2. How to visualise transpose of a matrix?


### References
* https://youtube.com/@visualkernel/videos
