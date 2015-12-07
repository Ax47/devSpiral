#include "lsstools.h"
#include <stdio.h>
#include <stdlib.h>
#include "master.h"
#include "slave.h"
#include "SpirallingControl.h"


extern char baud_rate;
extern SLAVES_conf slaves[SLAVE_NUMBER];

static int LSS_step = 1;
static int LSS_step_error = 0;
static int LSS_step_error_max = 2;


// A faire la gestion du send failed: no buffer space

int lsstools_loop(UNS8 nodeID, int error) {
    int i;
    SLAVES_conf slave = slaves[slave_get_index_with_node(nodeID)];
	if (error){
        LSS_step_error = 0;
        LSS_step = 1;
        return 0;
    }
    UNS8 LSS_mode = LSS_CONFIGURATION_MODE;
    UNS8 res;
	printf("ConfigureLSSNode step %d -> ",LSS_step);
	switch(LSS_step) { /*LSS=>put in configuration mode*/
        case 1:
            printf("LSS=>put in configuration mode\n");
            lsstools_config(nodeID, LSS_SM_SELECTIVE_VENDOR,&slave.vendor,0,0);
            lsstools_config(nodeID, LSS_SM_SELECTIVE_PRODUCT,&slave.product,0,0);
            lsstools_config(nodeID, LSS_SM_SELECTIVE_REVISION,&slave.revision,0,0);
            lsstools_config(nodeID, LSS_SM_SELECTIVE_SERIAL,&slave.serial,0,1);
        break;
        case 2: /** Vérification si le noeud est déja assigné **/
            lsstools_config(nodeID, LSS_INQ_NODE_ID,0,0,1);
        break;
        case 3: /** Assignation du noeud **/
            lsstools_config(nodeID, LSS_CONF_NODE_ID,&nodeID,0,1);
        break;

        case 4: /** Sauvegarde des données **/
            lsstools_config(nodeID, LSS_CONF_STORE,0,0,1);
        break;
        case 5: /** Sortie du mode configuration **/
            LSS_mode = LSS_WAITING_MODE;
            lsstools_config(nodeID, LSS_SM_GLOBAL,&LSS_mode,0,0);
            LSS_step = 1;
            LSS_step_error = 0;
            return 1;
        break;
    }
}

void lsstools_config(UNS8 nodeID, UNS8 command,void* dat2send1, void* dat2send2,int checkactive) {
    configNetworkNode(&SpirallingMaster_Data,command,dat2send1,dat2send2,NULL);
    if (checkactive) {
        UNS32 dat1;
        UNS8 dat2;
        UNS8 res = getConfigResultNetworkNode (&SpirallingMaster_Data, command, &dat1, &dat2);
        while (res == LSS_TRANS_IN_PROGRESS || res == LSS_RESET) {
            usleep(100000);
            res = getConfigResultNetworkNode (&SpirallingMaster_Data, command, &dat1, &dat2);
        }
        if (res == LSS_ABORTED_INTERNAL) {
			printf("Master : Failed in LSS comand %d.  Trying again\n", command);
            LSS_step_error++;
            if (LSS_step_error < LSS_step_error_max) {
                lsstools_loop(nodeID, 0);
            } else {
                lsstools_loop(nodeID, 1);
            }
        } else if (res == LSS_FINISHED) {
            LSS_step++;
            switch(command){
            case LSS_CONF_NODE_ID:
                switch(dat1){
                    case 0: printf("Node ID change succesful\n");break;
                    case 1: printf("Node ID change error:out of range\n");break;
                    case 0xFF:printf("Node ID change error:specific error\n");break;
                    default:break;
                }
                break;
            case LSS_CONF_BIT_TIMING:
                switch(dat1){
                    case 0: printf("Baud rate change succesful\n");break;
                    case 1: printf("Baud rate change error: change baud rate not supported\n");break;
                    case 0xFF:printf("Baud rate change error:specific error\n");break;
                    default:break;
                }
                break;
            case LSS_CONF_STORE:
                switch(dat1){
                    case 0: printf("Store configuration succesful\n");break;
                    case 1: printf("Store configuration error:not supported\n");break;
                    case 0xFF:printf("Store configuration error:specific error\n");break;
                    default:break;
                }
                break;
            case LSS_SM_SELECTIVE_SERIAL:
                printf("Slave in LSS CONFIGURATION state\n");
                break;
            case LSS_INQ_NODE_ID:
                printf("Slave nodeid %x\n", dat1);
                if (dat1 == nodeID) {
                    LSS_step = 5;
                }
                break;
            }
            printf("\n");
            lsstools_loop(nodeID, 0);
        }
    }
}

