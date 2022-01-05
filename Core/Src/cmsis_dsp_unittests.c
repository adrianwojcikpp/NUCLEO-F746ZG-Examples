/*
 * cmsis_dsp_unittests.c
 *
 *  Created on: 3 sty 2022
 *      Author: AW
 */

#include "main.h"

float SWV_VAR;

/**
 * @brief Root mean square error of vector 'y' with vector 'yref' as reference
 * @param[in] y    : Input vector
 * @param[in] yref : Reference vector
 * @param[in] len  : Vectors length
 * @return Root mean square error: sqrt(sum( (yref - y)^2 ))
 */
float32_t RMSE(float32_t* y, float32_t* yref, uint32_t len)
{
	float32_t sum_sq_error = 0;
	for(uint32_t i = 0; i < len; i++)
		sum_sq_error += (yref[i]-y[i])*(yref[i]-y[i]);
	return sqrtf(sum_sq_error / len);
}

/* FIR filter -----------------------------------*/
arm_fir_instance_f32 fir;

// Filter coefficients
const float32_t b[] = {
	#include "../../MATLAB/fir_b.csv"
};

// Filter state
float32_t fir_state[] = {
  #include "../../MATLAB/fir_state_init.csv"
};

#define FIR_NUM_TAPS (sizeof(b)/sizeof(float32_t))

/**
 * @brief FIR filter (arm_fir_f32 function) unit test based on
 *        MATLAB-generated data files.
 * @return Test status (arm_status)
 */
int8_t UNIT_TEST_FIR(void)
{
	/* LOCAL VARIABLES */
	arm_status status = ARM_MATH_TEST_FAILURE;
	float32_t rmse = 1.0;
	const float32_t rmse_max = 1e-6; //1e-10

	/* FIR INIT */
	arm_fir_init_f32(&fir, FIR_NUM_TAPS, b, fir_state, 1);

	float32_t x[] = {
		#include "../../MATLAB/fir_x.csv"
	};

	// Reference output
	float32_t yref[] = {
		#include "../../MATLAB/fir_yref.csv"
	};

	// Output
	#define FIR_NUM_SAMPLES (sizeof(x)/sizeof(float32_t))
	float32_t y[FIR_NUM_SAMPLES];

	/* DIGITAL SIGNAL FILTRATION */
	for(uint32_t i = 0; i < FIR_NUM_SAMPLES; i++)
	{
	  arm_fir_f32(&fir, &x[i], &y[i], 1);
	  SWV_VAR = y[i]; HAL_Delay(0); // for SWV
	}

	/* ROOT MEAN SQUARE ERROR */
	rmse = RMSE(y, yref, FIR_NUM_SAMPLES);

	/* RMSE TRESHOLD */
	if(rmse < rmse_max) status = ARM_MATH_SUCCESS;

	return status;
}

/* IIR filter -----------------------------------*/
arm_biquad_casd_df1_inst_f32 iir;

#define IIR_NUM_STAGES  1    //! Overall order is 2*IIR_NUM_STAGES

// Filter state
uint32_t biquad_state_hex[4*IIR_NUM_STAGES] = {
#include "../../MATLAB/biquad_state_init.csv"
};
float32_t *biquad_state = (float32_t*)biquad_state_hex;

// Filter coefficients
uint32_t biquad_coeffs_hex[5*IIR_NUM_STAGES] = {
#include "../../MATLAB/biquad_coeffs.csv"
};
float32_t *biquad_coeffs = (float32_t*)biquad_coeffs_hex;

/**
 * @brief IIR filter arm_biquad_cascade_df1_f32) unit test based on
 *        MATLAB-generated data files.
 * @return Test status (arm_status)
 */
int8_t UNIT_TEST_IIR(void)
{
  /* LOCAL VARIABLES */
  arm_status status = ARM_MATH_TEST_FAILURE;
  float32_t rmse = 1.0f;
  const float32_t rmse_max = 1e-10f;

  // Input
  uint32_t x_hex[] = {
  	#include "../../MATLAB/biquad_x.csv"
  };
  float32_t *x = (float32_t*)x_hex;

  // Reference output
  uint32_t yref_hex[] = {
  	#include "../../MATLAB/biquad_yref.csv"
  };
  float32_t *yref = (float32_t*)yref_hex;

  // Output
	#define IIR_NUM_SAMPLES (sizeof(x_hex)/sizeof(uint32_t))
  float32_t y[IIR_NUM_SAMPLES];

  /* IIR INIT */
  arm_biquad_cascade_df1_init_f32(&iir, IIR_NUM_STAGES, biquad_coeffs, biquad_state);

  /* TEST SIGNAL FILTRATION */
  for(uint32_t i = 0; i < IIR_NUM_SAMPLES; i++)
  {
  	arm_biquad_cascade_df1_f32(&iir, &x[i], &y[i], 1);
  	SWV_VAR = y[i]; HAL_Delay(0); // for SWV
  }

  /* ROOT MEAN SQUARE ERROR */
  rmse = RMSE(y, yref, IIR_NUM_SAMPLES);

  /* RMSE TRESHOLD */
  if(rmse < rmse_max) status = ARM_MATH_SUCCESS;

  return status;
}


/* PID controller -------------------------------*/
arm_pid_instance_f32 pid;

/**
 * @brief PID controller (arm_pid_f32) unit test based on
 *        MATLAB-generated data files.
 * @return Test status (arm_status)
 */
int8_t UNIT_TEST_PID(void)
{
  /* LOCAL VARIABLES */
  arm_status status = ARM_MATH_TEST_FAILURE;
  float32_t rmse = 1.0f;
  const float32_t rmse_max = 1e-5f; //1e-10f

  // Input
  uint32_t x_hex[] = {
		#include "../../MATLAB/pid_x.csv"
  };
  float32_t *x = (float32_t*)x_hex;

  // Reference output
  uint32_t yref_hex[] = {
    #include "../../MATLAB/pid_yref.csv"
  };
  float32_t *yref = (float32_t*)yref_hex;

  // Output
	#define PID_NUM_SAMPLES (sizeof(x_hex)/sizeof(uint32_t))
  float32_t y[PID_NUM_SAMPLES];

  // Controller gains
  uint32_t gains_hex[] = {
    #include "../../MATLAB/pid_gains.csv"
  };

  pid.Kp = ((float32_t*)gains_hex)[0];
  pid.Ki = ((float32_t*)gains_hex)[1];
  pid.Kd = ((float32_t*)gains_hex)[2];

  /* PID INIT */
  /** This function computes the structure fields: A0, A1 A2
   *  using the proportional gain (Kp), integral gain (Ki)
   *  and derivative gain (Kd); also sets the state variables to zeros.
   */
  arm_pid_init_f32(&pid, 1);

  /* CONTROL SIGNAL GENERATION */
  for(uint32_t i = 0; i < PID_NUM_SAMPLES; i++)
  {
  	y[i] = arm_pid_f32(&pid, x[i]);
  	SWV_VAR = y[i]; HAL_Delay(0); // for SWV
  }

  /* ROOT MEAN SQUARE ERROR */
  rmse = RMSE(y, yref, PID_NUM_SAMPLES);

  /* RMSE TRESHOLD */
  if(rmse < rmse_max) status = ARM_MATH_SUCCESS;

  return status;
}

/**
 * @brief Common CMSIS unit tests routine.
 */
void CMSIS_UnitTests(void)
{
  LCD_SetCursor(&hlcd1, 1, 0);
  LCD_printStr(&hlcd1, "TEST");

  float32_t cmplx_var[2] = {1.0f, 1.0f};
  float32_t cmplx_var_mag = 0.0f;

  arm_cmplx_mag_f32(cmplx_var, &cmplx_var_mag, 1);
  float32_t cmplx_var_mag_ref = sqrtf(2.0f);

  arm_status TEST_ARG = ARM_MATH_TEST_FAILURE;

  if(fabs(cmplx_var_mag - cmplx_var_mag_ref) < 1e-6)
  	TEST_ARG = ARM_MATH_SUCCESS;

  arm_status TEST_FIR = UNIT_TEST_FIR();
  arm_status TEST_IIR = UNIT_TEST_IIR();
  arm_status TEST_PID = UNIT_TEST_PID();

  if(TEST_ARG != ARM_MATH_SUCCESS)
  	LCD_printStr(&hlcd1, " ARG");

  if(TEST_FIR != ARM_MATH_SUCCESS)
  	LCD_printStr(&hlcd1, " FIR");

  if(TEST_IIR != ARM_MATH_SUCCESS)
  	LCD_printStr(&hlcd1, " FIR");

  if(TEST_PID != ARM_MATH_SUCCESS)
  	LCD_printStr(&hlcd1, " PID");

  if(TEST_ARG == ARM_MATH_SUCCESS && TEST_FIR== ARM_MATH_SUCCESS &&
  	 TEST_IIR == ARM_MATH_SUCCESS && TEST_PID == ARM_MATH_SUCCESS )
  {
  	LCD_printStr(&hlcd1, " ALL SUCESS");
  	LED_On(&hledg2);
  }
  else
  {
  	LCD_printStr(&hlcd1, " FAIL");
  	LED_On(&hledr2);
  }
}
