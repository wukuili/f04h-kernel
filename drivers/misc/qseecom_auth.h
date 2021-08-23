/*
 * ==========================================================================
 * COPYRIGHT(C) FUJITSU LIMITED 2014  All Rights Reserved.
 * --------------------------------------------------------------------------
 * Products  :
 * Source    : qseecom_auth.h
 * Revision  :
 * Date      :
 * Author    :
 * --------------------------------------------------------------------------
 * Abstract  : header file for client authentication process
 * --------------------------------------------------------------------------
 *
 *                       EDIT HISTORY FOR MODULE
 *
 * when       who      what, where, why
 * ---------- -------- ------------------------------------------------------
 * 2014/5/12  F.Sato  New preparation.
 * 2015/3/12  Fujitsu Support FIDO.
 *
 */

#ifndef __QSEECOM_AUTH_H__
#define __QSEECOM_AUTH_H__
/* This file is used to define the client authentication criteria
 * and valid id list array.
 * Two dimension array "authentication_array" holds the information of 
 * "Used parameters", "user ID", "group ID" and "hash value" in each 
 * client ID array.
 * Definition is : 
 * const int authentication_array[CLIENT_ID][CRITERIA] = {
 *   {"Used parameters", "user ID", "group ID", "hash value"}
 *                                :    
 *                                :
 *    }
 * First array number means client ID stands for client process such as 
 * LifeLog, Fullseg, Felica or any other application or process.
 * (In 14-2nd development, the only client process is LifeLog application)
 * This array can be extend if new client process wants to be added.
 * Second array have four elements. First constant "Used parameter"
 * means which parameters will be used for authenticattion.
 * Each bit of this value indicates "USE" or "NOT USE" for each parameters, 
 * user ID, group ID and hash value.
 * For example, the value of "Used parameters" is 0x03, group ID and 
 * hash value will be referred and evaluated for authentication.
 * Second, third, and fourth constants are required authentication value 
 * of user ID, group ID, hash value for each.
 * Client process must have this user ID, or belong this group, or have this 
 * hash value to access Trust Zone.
 * 
 * The tzc_valid_id_list is also implemented in order to record process_ID 
 * and app_ID of the client process. Process ID is put to the process, when 
 * it started. app_id is put when TZ application is started.
 * If client authentication process is accepted, process_ID will be registerd 
 * in tzc_valid_id_list from low array number. Then, if "qseecom_start_app" 
 * is called from "fjsec_tz_cipher" app, check if process id of the 
 * client is registered in tzc_valid_id_list. If yes, TZapp can be loaded.
 * Then after the TZapp loaded, app_id will be registered in tzc_valid_id_list 
 * in the same array which process_id have been registered. 
 * When "qseecom_send_modfd_cmd" is called, check if 
 * app_id is registerd in tzc_valid_id_list. If yes, additionaly check if 
 * process_id in same array is agree with accessing process's one. 
 * After thise confirmaion, client process can send command to TZ application.
*/

#define TZAPP_CLIENT_MAX 16
// 15-1st_LIFELOG_001 Mod start
#define LOCAL_CLIENT_MAX 2
// 15-1st_LIFELOG_001 Mod end

#define CHECK_USER_ID    0x04
#define CHECK_GROUP_ID   0x02
#define CHECK_HASH_VALUE 0x01

enum AUTH_ARRAY_CONT{
	AUTH_TYPE = 0,
	USER_ID_ARRAY,
	GROUP_ID_ARRAY,
	HASH_ARRAY
};

enum VALID_ID_LIST_CONT{
	PROCESS_ID_LIST = 0,
	TZAPP_ID_LIST
};

const int authentication_array[][4] = {
	/*  {Use parameter, user ID, group ID, hash} */
	{CHECK_GROUP_ID, -1, 5050, -1},		/* 0 : LIFELOG      */
	{CHECK_GROUP_ID, -1, 1006, -1},		/* 1 : FIDO(Camera) */
	{-1, -1, -1, -1}					/* 2 : CLIENT MAX   */
};

static int tzc_valid_id_list[LOCAL_CLIENT_MAX + 1][TZAPP_CLIENT_MAX + 1][2] ={{{0}}}; /*process_ID, TZapp_ID */

const char tzc_name[][32] = {"fjsec_tz_cipher",
							 "fj_iris_verifier",
							 ""
							};

#endif /* __QSEECOM_AUTH_H__ */
