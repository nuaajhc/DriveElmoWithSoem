//This file drives two motors to move in csp mode and this version separates data processing from message sending and receiving. 


#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatdc.h>
#include <ethercatcoe.h>
#include <ethercatfoe.h>
#include <ethercatconfig.h>
#include <ethercatprint.h>

#define EC_TIMEOUTMON 500
#define INITIAL_POS 0

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

//RPDO
struct TorqueOut {
    int32 target_position;
    uint32 digital_output;
    uint16 control_word;
};

//TPDO
struct TorqueIn {
    int32 position_actual_value;
    uint32 digital_inputs;
    uint16 status_word;
};

target = (struct TorqueOut*)(ec_slave[1].outputs);
val = (struct TorqueIn*)(ec_slave[1].inputs);

target2 = (struct TorqueOut*)(ec_slave[2].outputs);
val2 = (struct TorqueIn*)(ec_slave[2].inputs);

/* helper macros*/

#define READ(slaveId, idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, (unsigned int)buf, comment);    \
     }

#define WRITE(slaveId, idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

#define CHECKERROR(slaveId)   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}

void CheckSendFlag()
{
    int old_flag = SendFlag;
    //printf("Flag: %d\n", old_flag);
    while (1)
    {   
        //printf("Flag: %d\n", SendFlag);
        if (SendFlag != old_flag){
            break;
        }
    }
}

void* ecatprint(void* ptr)
{
    int slave;
    while (1)
    {
        //printf("Main Outputs:  Target: 0x%x, control: 0x%x\n", target->torque, target->status);
        //printf("Main Inputs:  Target: 0x%x, control: 0x%x\n", val->position, val->torque);
        //printf("Flag: %d\n", SendFlag);
        SendFlag = !SendFlag;
        //printf("Flag: %d\n", SendFlag);
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        usleep(10000);
    }
}

void simpletest(char* ifname)
{
    int i, j, oloop, iloop, wkc_count, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    int32 sbuf32;

    struct TorqueIn* val;
    struct TorqueOut* target;

    struct TorqueIn* val2;
    struct TorqueOut* target2;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i = 1; i <= ec_slavecount; i++) {
                printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true" : "false");

                /** CompleteAccess disabled for Elmo driver */
                ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            /** set PDO mapping */
            /** opMode: 8  => Position profile */
            for (int i = 1; i <= ec_slavecount; i++) {
                WRITE(i, 0x6060, 0, buf8, 8, "OpMode");
                READ(i, 0x6061, 0, buf8, "OpMode display");


                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            int32 ob2; int os;
            for (int i = 1; i <= ec_slavecount; i++) {
                os = sizeof(ob2); ob2 = 0x16000001;
                ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                os = sizeof(ob2); ob2 = 0x1a000001;
                ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            /** if CA disable => automapping works */
            ec_config_map(&IOmap);

            // show slave info
            for (int i = 1; i <= ec_slavecount; i++) {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                    i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                    ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            }

            /** disable heartbeat alarm */
            for (int i = 1; i <= ec_slavecount; i++) {
                READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
            }

            printf("Slaves mapped, state to SAFE_OP.\n");

            int timestep = 200;

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

            /** old SOEM code, inactive */
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 20) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 20) iloop = 8;

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            
            for (int i = 1; i <= ec_slavecount; i++) {
                READ(i, 0x6083, 0, buf32, "Profile acceleration");
                READ(i, 0x6084, 0, buf32, "Profile deceleration");
                READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
            }

            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */
                for (int i = 1; i <= ec_slavecount; i++) {


                    READ(i, 0x6064, 0, sbuf32, "*position actual value*");

                    READ(i, 0x6041, 0, buf16, "*status word*");
                    if (buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*"); usleep(100000);
                        READ(i, 0x6041, 0, buf16, "*status word*");
                    }


                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6060, 0, buf8, 6, "OpMode"); usleep(100000);
                    WRITE(i, 0x6098, 0, buf8, 35, "homing method"); usleep(100000);
                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(100000);
                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(100000);
                    WRITE(i, 0x6040, 0, buf16, 31, "*control word*"); usleep(100000);
                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(100000);


                    WRITE(i, 0x6060, 0, buf8, 8, "OpMode");

                    READ(i, 0x6064, 0, sbuf32, "*position actual value*");


                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*");

                    CHECKERROR(i);
                    READ(i, 0x1a0b, 0, buf8, "OpMode Display");

                    READ(i, 0x1001, 0, buf8, "Error");
                }
                int reachedInitial = 0;
                int reachedInitial2 = 0;
                
                pthread_create(&thread2, NULL, &ecatprint, (void(*)) & ctime);
                /* cyclic loop for two slaves*/
                for (i = 1; i <= 500; i++)
                {

                    if (wkc >= expectedWKC) {
                        printf("Processdata cycle %4d, WKC %d,", i, wkc);
                        printf("  pos: 0x%x, stat: 0x%x,", val->position_actual_value, val->status_word);

                        switch (target->control_word) {
                        case 0:
                            target->control_word = 6;
                            break;
                        case 6:
                            target->control_word = 7;
                            break;
                        case 7:
                            target->control_word = 15;
                            break;
                        case 128:
                            target->control_word = 0;
                            break;
                        default:
                            if (val->status_word >> 3 & 0x01) {
                                READ(1, 0x1001, 0, buf8, "Error");
                                target->control_word = 128;
                            }
                        }

                        switch (target2->control_word) {
                        case 0:
                            target2->control_word = 6;
                            break;
                        case 6:
                            target2->control_word = 7;
                            break;
                        case 7:
                            target2->control_word = 15;
                            break;
                        case 128:
                            target2->control_word = 0;
                            break;
                        default:
                            if (val2->status_word >> 3 & 0x01) {
                                READ(2, 0x1001, 0, buf8, "Error");
                                target2->control_word = 128;
                            }
                        }
                        if (reachedInitial == 0  && (val->status_word & 0x0fff) == 0x0237) {
                            reachedInitial = 1;
                        }

                        if (reachedInitial2 == 0  && (val2->status_word & 0x0fff) == 0x0237) {
                            reachedInitial2 = 1;
                        }
                        
                        CheckSendFlag();

                        if ((val->status_word & 0x0fff) == 0x0237 && reachedInitial) {
                            target->target_position = (int32)(sin(i / 100.) * (50000));
                        }

                        if ((val2->status_word & 0x0fff) == 0x0237 && reachedInitial2) {
                            target2->target_position = (int32)(sin(i / 100.) * (50000));
                        }

                        printf("  Target: 0x%x, control: 0x%x", target->target_position, target->control_word);

                        printf("\r");
                        needlf = TRUE;
                    }
                    //usleep(timestep);
                }
                inOP = FALSE;
            }
            
            
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for (i = 1; i <= ec_slavecount; i++)
                {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            printf("\nRequest init state for all slaves\n");
            for (int i = 1; i <= ec_slavecount; i++) {
                WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }

            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

void* ecatcheck(void* ptr)
{
    int slave;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf(".");
        }
        usleep(250);
    }
}

int main(int argc, char* argv[])
{
    int iret1;
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        iret1 = pthread_create(&thread1, NULL, &ecatcheck, (void(*)) & ctime); // (void) &ctime
        /* start cyclic part */
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    printf("End program\n");
    return (0);
