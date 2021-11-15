/**
  ******************************************************************************
  * @file    common.h
  * @author  AW       Adrian.Wojcik@put.poznan.pl
  * @version 1.0
  * @date    15-Nov-2021
  * @brief   Common components definitions and macroinstructions.
  *
  ******************************************************************************
  */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/**
 * @brief Linear transformation of 'x' from <amin, amax> to <bmin, bmax>.
 * @param[in] x Input variable
 * @param[in] amin Minimum of input range
 * @param[in] amax Maximum of input range
 * @param[in] bmin Minimum of output range
 * @param[in] bmax Maximum of output range
 * @return Scaled output variable in <bmin, bmax> range
 */
#define LINEAR_TRANSFORM(x,amin,amax,bmin,bmax) (((x-amin)/(amax-amin))*(bmax-bmin)+bmin)

#endif /* INC_COMMON_H_ */
