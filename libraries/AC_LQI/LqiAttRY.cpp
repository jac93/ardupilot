
#include "LqiAttRY.h"

void LQRcontrolRY::step(float rate_des[3],float rate_meas[3],float (&output)[2],float dt) 
{
    //update rate error
    err_rate[0] = rate_des[0]-rate_meas[0];
    err_rate[1] = rate_des[1]-rate_meas[1];

    //update integral of error
    update_integral_err(err_rate,dt);

    //calculate roll, yaw torques based on lqi matrix
    matrix_multiply(Klqi,err_rate,output);

}

void LQRcontrolRY::matrix_multiply(float A[2][4], float x[4], float u[2])
{
    for(int i = 0; i<2;i++)
    {
        u[i] = 0;

        for(int j=0; j<4;j++)
        {
            u[i]+= A[i][j]*x[j];
        }
    }


}

// void LQRcontrolRY::set_Klqi(Kgains gains)
// {
//     Klqi[0][0] = gains.kxp;
//     Klqi[0][1] = gains.kxr;
//     Klqi[0][2] = gains.kxp_int;
//     Klqi[0][3] = gains.kxr_int;

//     Klqi[1][0] = gains.kzp;
//     Klqi[1][1] = gains.kzr;
//     Klqi[1][2] = gains.kzp_int;
//     Klqi[1][3] = gains.kzr_int;



// }

void LQRcontrolRY::initialize(void)
{
    // set_Kgains_default(gains);
    // set_Klqi(gains);

}

// void LQRcontrolRY::set_Kgains_default(Kgains gains)
// {
//     gains.kxp = 0.4539;
//     gains.kxr = -0.0587;
//     gains.kxp_int = 0.3794;
//     gains.kxr_int = -0.0668;
    
//     gains.kzp = 0.1143;
//     gains.kzr = 1.3073;
//     gains.kzp_int = 0.0445;
//     gains.kzr_int = 0.5691;
// }

void LQRcontrolRY::update_integral_err(float (&error)[4],float dt)
{
    for(int j=2;j<4;j++)
    {
        if (dt>=0) 
        {
   
                error[j] += error[j-2] * dt;
            
        } 
        else 
        {
            error[j] = 0.0f;
        }
    }
}
