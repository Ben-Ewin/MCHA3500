#include <stddef.h>
#include "stm32f4xx_hal.h"
#include "arm_math.h" /* Include STM32 DSP matrix libraries */
#include "qpas_sub_noblas.h"
#include "controller.h"

// Variables for QP solver
int numits,numadd,numdrop;

// Define actuator limits
#define u_min -1.0
#define u_max 1.0
#define delta_u_min -0.1
#define delta_u_max 0.1

/* Define control arrays */
// Hessian
static float32_t ctrl_H_f32[CTRL_N_HORIZON*CTRL_N_HORIZON] =
{
    0.0027851f, 0.00073969f, 0.00069542f, 0.00065327f, 0.00061308f, 0.00057471f, 0.00053801f, 0.00050285f, 0.0004691f, 0.00043662f, 
    0.00073969f, 0.0027011f, 0.00066012f, 0.00061998f, 0.00058172f, 0.00054521f, 0.0005103f, 0.00047687f, 0.0004448f, 0.00041394f, 
    0.00069542f, 0.00066012f, 0.0026257f, 0.0005886f, 0.00055215f, 0.00051737f, 0.00048415f, 0.00045234f, 0.00042184f, 0.00039251f, 
    0.00065327f, 0.00061998f, 0.0005886f, 0.0025579f, 0.00052428f, 0.00049113f, 0.00045947f, 0.00042919f, 0.00040015f, 0.00037225f, 
    0.00061308f, 0.00058172f, 0.00055215f, 0.00052428f, 0.0024969f, 0.00046641f, 0.00043622f, 0.00040735f, 0.00037969f, 0.00035313f, 
    0.00057471f, 0.00054521f, 0.00051737f, 0.00049113f, 0.00046641f, 0.002442f, 0.00041432f, 0.00038678f, 0.0003604f, 0.00033508f, 
    0.00053801f, 0.0005103f, 0.00048415f, 0.00045947f, 0.00043622f, 0.00041432f, 0.0023926f, 0.00036741f, 0.00034222f, 0.00031807f, 
    0.00050285f, 0.00047687f, 0.00045234f, 0.00042919f, 0.00040735f, 0.00038678f, 0.00036741f, 0.002348f, 0.00032511f, 0.00030204f, 
    0.0004691f, 0.0004448f, 0.00042184f, 0.00040015f, 0.00037969f, 0.0003604f, 0.00034222f, 0.00032511f, 0.0023079f, 0.00028696f, 
    0.00043662f, 0.00041394f, 0.00039251f, 0.00037225f, 0.00035313f, 0.00033508f, 0.00031807f, 0.00030204f, 0.00028696f, 0.0022716f,
};

// f bar
static float32_t ctrl_fBar_f32[CTRL_N_HORIZON*CTRL_N_STATE] =
{
    -0.05144f, -0.57423f, -0.15372f, -0.089018f, 
    -0.047343f, -0.54086f, -0.14464f, -0.083761f, 
    -0.043459f, -0.50899f, -0.13597f, -0.078749f, 
    -0.039776f, -0.47854f, -0.12768f, -0.073965f, 
    -0.036284f, -0.4494f, -0.11975f, -0.069395f, 
    -0.03297f, -0.42149f, -0.11214f, -0.065023f, 
    -0.029826f, -0.39471f, -0.10485f, -0.060834f, 
    -0.026842f, -0.36898f, -0.097834f, -0.056815f, 
    -0.024007f, -0.34421f, -0.091084f, -0.05295f, 
    -0.021315f, -0.32031f, -0.084574f, -0.049228f,
};

// f
static float32_t ctrl_f_f32[CTRL_N_HORIZON] =
{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
};

// State vector
static float32_t ctrl_xHat_f32[CTRL_N_STATE] =
{
    0.0,
    0.0,
    0.0,
    0.0,
};

// Control
static float32_t ctrl_u_f32[CTRL_N_INPUT] =
{
    0.0,
};

// U star
static float32_t ctrl_Ustar_f32[CTRL_N_HORIZON*CTRL_N_INPUT] =
{
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
};

// Constraints
static float ctrl_A_f32[CTRL_N_INEQ_CONST*CTRL_N_HORIZON] =
{
    // 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 
    // -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 
    // 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f,

    1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 0.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 
};

static float ctrl_b_f32[CTRL_N_INEQ_CONST] =
{
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f, 
    0.1f,
};

static float ctrl_xl_f32[CTRL_N_LB_CONST] =
{
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f, 
    -1.0f,
};

static float ctrl_xu_f32[CTRL_N_UB_CONST] =
{
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f, 
    1.0f,
};

static float ctrl_lm_f32[CTRL_N_EQ_CONST+CTRL_N_INEQ_CONST+CTRL_N_LB_CONST+CTRL_N_UB_CONST] =
{

};

/* Define control matrix variables */
// rows, columns, data array
arm_matrix_instance_f32 ctrl_fBar = {CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_fBar_f32};
arm_matrix_instance_f32 ctrl_f = {CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32};
arm_matrix_instance_f32 ctrl_xHat = {CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32};
arm_matrix_instance_f32 ctrl_u = {CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32};

/* Control functions */
void ctrl_init(void)
{
    arm_mat_init_f32(&ctrl_fBar, CTRL_N_HORIZON, CTRL_N_STATE, (float32_t *)ctrl_fBar_f32);
    arm_mat_init_f32(&ctrl_f, CTRL_N_HORIZON, 1, (float32_t *)ctrl_f_f32);
    arm_mat_init_f32(&ctrl_xHat, CTRL_N_STATE, 1, (float32_t *)ctrl_xHat_f32);
    arm_mat_init_f32(&ctrl_u, CTRL_N_INPUT, 1, (float32_t *)ctrl_u_f32);
}

/* Update state vector elements */
void ctrl_set_x1(float x1)
{
    // Update state x1
    ctrl_xHat_f32[0] = x1;
};

void ctrl_set_x2(float x2)
{
    // Update state x2
    ctrl_xHat_f32[1] = x2;
};

void ctrl_set_x3(float x3)
{
    // Update state x3
    ctrl_xHat_f32[2] = x3;
};

void ctrl_set_x4(float x4)
{
    // Update state x4
    ctrl_xHat_f32[3] = x4;
};

/* Get the current control output */
float getControl(void)
{
    return ctrl_u_f32[0];
};

/* Update control output */
void ctrl_update(void)
{
    // TODO: Compute f vector
    arm_mat_mult_f32(&ctrl_fBar, &ctrl_xHat, &ctrl_f);

    // TODO: Update b vector
    ctrl_b_f32[0] = ctrl_u_f32[0] + delta_u_max;
    ctrl_b_f32[CTRL_N_HORIZON] = -ctrl_u_f32[0] - delta_u_min;

    // TODO: Solve for optimal inputs over control horizon
    // qpas_sub_noblas(CTRL_N_HORIZON, CTRL_N_EQ_CONST, CTRL_N_INEQ_CONST, CTRL_N_LB_CONST, CTRL_N_UB_CONST, 
    //     &ctrl_H_f32, &ctrl_f_f32, &ctrl_A_f32, &ctrl_b_f32, &ctrl_xl_f32, &ctrl_xu_f32, &ctrl_Ustar_f32, 
    //     &ctrl_lm_f32, 0,
    //     &numits, &numadd, &numdrop);

    // TODO: Extract first control term
    ctrl_u_f32[0] = ctrl_Ustar_f32[0];

    /* Print functions for debugging. Uncomment to use */
    // printmatrix (CTRL_N_HORIZON,CTRL_N_HORIZON,ctrl_H_f32,CTRL_N_HORIZON,"H");
    // printvector (CTRL_N_HORIZON, ctrl_f_f32, "f");
}