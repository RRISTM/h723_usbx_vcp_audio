
/* Includes ------------------------------------------------------------------*/
#include "ux_device_audio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
UX_DEVICE_CLASS_AUDIO *audio;
UX_DEVICE_CLASS_AUDIO_STREAM *playback_stream;
UX_DEVICE_CLASS_AUDIO20_CONTROL audio_control[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
uint16_t fillArray(uint16_t* buffer,uint16_t size,uint16_t firstSample);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

VOID USBD_AUDIO_Activate(VOID *audio_instance){
	  /* Save the audio instance */
	  audio = (UX_DEVICE_CLASS_AUDIO *)audio_instance;

	  /* Get the audio stream instance */
	  ux_device_class_audio_stream_get(audio, 0, &playback_stream);
}
VOID USBD_AUDIO_Deactivate(VOID *audio_instance){
	  /* Reset the Audio instance */
	  audio = UX_NULL;
}
VOID USBD_AUDIO_Request(UX_DEVICE_CLASS_AUDIO *audio_instance,
        UX_SLAVE_TRANSFER *transfer){
UINT status  = UX_SUCCESS;

  /* USER CODE BEGIN USBD_AUDIO_ControlProcess */
   UX_DEVICE_CLASS_AUDIO20_CONTROL_GROUP group;

   group.ux_device_class_audio20_control_group_controls_nb = 1;
   group.ux_device_class_audio20_control_group_controls = audio_control;

   status = ux_device_class_audio20_control_process(audio, transfer, &group);

   if (status == UX_SUCCESS)
   {

     switch(audio_control[0].ux_device_class_audio20_control_changed)
     {
       case UX_DEVICE_CLASS_AUDIO20_CONTROL_MUTE_CHANGED:

        /* Update mute state */
//        PlaybackAudioDescription.audio_mute = audio_control[0].ux_device_class_audio20_control_mute[0];

        /* Mute/Unmute audio codec volume */
//        AUDIO_SpeakerMute(PlaybackAudioDescription.audio_mute);

        break;

       case UX_DEVICE_CLASS_AUDIO20_CONTROL_VOLUME_CHANGED:

        /* Update current volume */
//        PlaybackAudioDescription.audio_volume_db_256 = (audio_control[0].ux_device_class_audio20_control_volume[0] + 1);

        /* Set audio codec volume */
//        AUDIO_SpeakerSetVolume(PlaybackAudioDescription.audio_volume_db_256);

        break;

       case UX_DEVICE_CLASS_AUDIO20_CONTROL_FREQUENCY_CHANGED:
        break;

       default:
        break;
     }
   }

}

uint16_t fillArray(uint16_t* buffer,uint16_t size,uint16_t firstSample){
	uint32_t i;
	for(i=0;i<(size/2);i++){
		buffer[i]=firstSample+i;
	}
	return buffer[(size/2)-1] + 1;
}

uint16_t buffer[4096]={1};
uint16_t lastSample=0;
VOID USBD_AUDIO_Write_Done(VOID *audio_stream,ULONG actual_length){
	UINT retVal =  ux_device_class_audio_frame_write(audio_stream,buffer,768);
	 HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_0);
  if (retVal!=UX_SUCCESS){
    __NOP();
  }else{
	  lastSample=fillArray(buffer, 768, lastSample);

  }
	// buffer[0]++;

}

VOID USBD_AUDIO_Write_Change(VOID *audio_stream,ULONG alternate_setting){
/* Do nothing if alternate setting is 0 (stream closed) */
  if (alternate_setting == 0)
  {
    return;
  }
  lastSample=fillArray(buffer, 768, lastSample);
  //     /* Start reception (stream opened) */
  UINT retVal =ux_device_class_audio_frame_write(audio_stream,buffer,768);
   if (retVal!=UX_SUCCESS){
     __NOP();
   }
   ux_device_class_audio_transmission_start(audio_stream);
  //  buffer[0]++;
}


void USBD_AUDIO_Handle_SOF(){
  if(audio != UX_NULL){
          /* Start reception (stream opened) */
//    ux_device_class_audio_frame_write(playback_stream,buffer,256);
//    ux_device_class_audio_transmission_start(playback_stream);
  }

}

#define USBD_AUDIO_SPEAKER_MUTED                  0
#define USBD_AUDIO_VOLUME_SPEAKER_MIN_DB_256      -6400 /* -25db == -25*256 = -6400 db */
#define USBD_AUDIO_VOLUME_SPEAKER_MAX_DB_256      1536  /* 6db == 6*256 = 1536 db */
#define USBD_AUDIO_VOLUME_SPEAKER_RES_DB_256      1     /* 1db == 1*256 = 256 db */
#define USBD_AUDIO_VOLUME_SPEAKER_DEFAULT_DB_256  256   /* 1db */
/**
  * @brief  USBD_AUDIO_SetControlValues
  *         This function is invoked to Set audio control values.
  * @param  none
  * @retval none
  */
VOID USBD_AUDIO_SetControlValues(VOID)
{
  /* USER CODE BEGIN USBD_AUDIO_SetControlValues */

  /* Fill audio control structure */
  audio_control[0].ux_device_class_audio20_control_cs_id = 25;
//  audio_control[0].ux_device_class_audio20_control_sampling_frequency = 192000U;
  audio_control[0].ux_device_class_audio20_control_sampling_frequency = 48000;
  audio_control[0].ux_device_class_audio20_control_fu_id = 21;
  audio_control[0].ux_device_class_audio20_control_mute[0] = USBD_AUDIO_SPEAKER_MUTED;
  audio_control[0].ux_device_class_audio20_control_volume_min[0] = USBD_AUDIO_VOLUME_SPEAKER_MIN_DB_256;
  audio_control[0].ux_device_class_audio20_control_volume_max[0] = USBD_AUDIO_VOLUME_SPEAKER_MAX_DB_256;
  audio_control[0].ux_device_class_audio20_control_volume_res[0] = USBD_AUDIO_VOLUME_SPEAKER_RES_DB_256;
  audio_control[0].ux_device_class_audio20_control_volume[0] = USBD_AUDIO_VOLUME_SPEAKER_DEFAULT_DB_256;

  /* USER CODE END USBD_AUDIO_SetControlValues */

  return;
}

/* USER CODE END 0 */


