#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.1415927
#endif

// Multiplies 3*3 times 3*1 matrix
// Fills in the value inside newly created value of new_pos
void matrix_multiply(double cur_pos[4][4], double new_pos[4][4], double matrix[4][4]){
        for(int i = 0; i < 4 ; i++){
            for(int j = 0; j < 4; j++){
                new_pos[i][j] = cur_pos[i][0]*matrix[0][j] + cur_pos[i][1]*matrix[1][j] + cur_pos[i][2]*matrix[2][j] + cur_pos[i][3]*matrix[3][j];
            }
        }
};

// k == 1 is the rotation around X, k == 2 is the rotation around Y
// The remaining means rotation around Z
// Fills in the value in provided pointer of matrix_val
void return_matrix(int k, double theta, double matrix_val[4][4]){
    if (k == 1){
        // Rotation around X Matrix
        matrix_val[0][0] = 1;
        matrix_val[0][1] = 0;
        matrix_val[0][2] = 0;
        matrix_val[0][3] = 0;
        matrix_val[1][0] = 0;
        matrix_val[1][1] = cos(theta);
        matrix_val[1][2] = -sin(theta);
        matrix_val[1][3] = 0;
        matrix_val[2][0] = 0;
        matrix_val[2][1] = sin(theta);
        matrix_val[2][2] = cos(theta); 
        matrix_val[2][3] = 0;
        matrix_val[3][0] = 0;
        matrix_val[3][1] = 0;
        matrix_val[3][2] = 0;
        matrix_val[3][3] = 1;
    }
    else if (k == 2){
        // Rotation around Y
        matrix_val[0][0] = cos(theta);
        matrix_val[0][1] = 0;
        matrix_val[0][2] = sin(theta);
        matrix_val[0][3] = 0;
        matrix_val[1][0] = 0;
        matrix_val[1][1] = 1;
        matrix_val[1][2] = 0;
        matrix_val[1][3] = 0;
        matrix_val[2][0] = -sin(theta);
        matrix_val[2][1] = 0;
        matrix_val[2][2] = cos(theta); 
        matrix_val[2][3] = 0;
        matrix_val[3][0] = 0;
        matrix_val[3][1] = 0;
        matrix_val[3][2] = 0;
        matrix_val[3][3] = 1;
    }
    else{
        // Rotation around Z
        matrix_val[0][0] = cos(theta);
        matrix_val[0][1] = -sin(theta);
        matrix_val[0][2] = 0;
        matrix_val[0][3] = 0;
        matrix_val[1][0] = sin(theta);
        matrix_val[1][1] = cos(theta);
        matrix_val[1][2] = 0;
        matrix_val[1][3] = 0;
        matrix_val[2][0] = 0;
        matrix_val[2][1] = 0;
        matrix_val[2][2] = 1; 
        matrix_val[2][3] = 0;
        matrix_val[3][0] = 0;
        matrix_val[3][1] = 0;
        matrix_val[3][2] = 0;
        matrix_val[3][3] = 1;
    }    
};

fwd_kin(theta, x)
double *theta;
double x[3];
{
    // These are the values given in question; may be changed as necessary 
    double l0 = 0.25;
    double l1 = 0.2;
    double l2 = 0.2;
    double l3 = 0.15;
    double d1 = -0.04;
    double d2 = 0.04;
    double d3 = -0.04;
    double d4 = -0.04;

    // The robot is now at the position on this location 
    double a0[4][4] = {
                            {1,0,0,0},
                            {0,1,0,0},
                            {0,0,1,l0},
                            {0,0,0,1}
                            };
    double a1[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };

    // Rotation around Z with Theta Zero 
    double rotation_z_theta_zero[4][4];
    return_matrix(0,theta[0],rotation_z_theta_zero);
    matrix_multiply(a0,a1,rotation_z_theta_zero);

    double a2[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };

    // Rotation around positive Y with Theta One 
    double rotation_y_theta_one[4][4];
    return_matrix(2,theta[1],rotation_y_theta_one);
    matrix_multiply(a1,a2,rotation_y_theta_one);
    
    // Translation with d1
    double translate_d1[4][4] = {
                            {1,0,0,0},
                            {0,1,0,d1},
                            {0,0,1,0},
                            {0,0,0,1}
                        };
    double a3[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a2,a3,translate_d1);

    // Translating with l1
    double translate_l1[4][4] = {
                            {1,0,0,l1},
                            {0,1,0,0},
                            {0,0,1,0},
                            {0,0,0,1}
                        };
    double a4[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a3,a4,translate_l1);

    // Rotate around Y with Theta 2
    double a5[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    double rotation_y_theta_two[4][4];
    return_matrix(2,theta[2],rotation_y_theta_two);
    matrix_multiply(a4,a5,rotation_y_theta_two);

    // Translation with d2
    double translate_d2[4][4] = {
                            {1,0,0,0},
                            {0,1,0,d2},
                            {0,0,1,0},
                            {0,0,0,1}
                        };
    double a6[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a5,a6,translate_d2);

    // Translating with l2
    double translate_l2[4][4] = {
                            {1,0,0,l2},
                            {0,1,0,0},
                            {0,0,1,0},
                            {0,0,0,1}
                        };
    double a7[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a6,a7,translate_l2);

    // Rotate around Y with Theta 3
    double a8[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    double rotation_y_theta_three[4][4];
    return_matrix(2,theta[3],rotation_y_theta_three);
    matrix_multiply(a7,a8,rotation_y_theta_three);

    // Translation  around d3
    double translate_d3[4][4] = {
                            {1,0,0,0},
                            {0,1,0,d3},
                            {0,0,1,0},
                            {0,0,0,1}
                        };
    double a9[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a8,a9,translate_d3);

    // Translation  around d4
    double translate_d4[4][4] = {
                            {1,0,0,0},
                            {0,1,0,0},
                            {0,0,1,d4},
                            {0,0,0,1}
                        };
    double a10[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a9,a10,translate_d4);

    // Adding on l3
    double translate_l3[4][4] = {
                            {1,0,0,l3},
                            {0,1,0,0},
                            {0,0,1,0},
                            {0,0,0,1}
                        };
    double a11[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    matrix_multiply(a10,a11,translate_l3);

    // Rotating by theta 4 on Y-axis 
    double a12[4][4] = {
                        {1,0,0,0},
                        {0,1,0,0},
                        {0,0,1,0},
                        {0,0,0,0}
                        };
    double rotation_y_theta_four[4][4];
    return_matrix(2,theta[4],rotation_y_theta_four);
    matrix_multiply(a11,a12,rotation_y_theta_four);

    x[0] = a12[0][3];
    x[1] = a12[1][3];
    x[2] = a12[2][3];
}


inv_kin(x, theta)
double *x;
double theta[6];
{

}