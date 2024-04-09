/*
 * gps.h
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 * Functions prototypes necessary for managing GPS communication with the SIM7600X module
 *
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "send_data.h"

void start_gps(void);
void end_gps(void);
char* get_gps_info(void);

#endif /* INC_GPS_H_ */
