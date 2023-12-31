/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UX_DEVICE_AUDIO_H__
#define __UX_DEVICE_AUDIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ux_api.h"
#include "ux_device_class_audio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ux_device_class_audio.h"
#include "ux_device_class_audio20.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
VOID USBD_AUDIO_Activate(VOID *audio_instance);
VOID USBD_AUDIO_Deactivate(VOID *audio_instance);
VOID USBD_AUDIO_Request(UX_DEVICE_CLASS_AUDIO *audio_instance, UX_SLAVE_TRANSFER *transfer);
VOID USBD_AUDIO_Write_Done(VOID *audio_stream,ULONG actual_length);
VOID USBD_AUDIO_Write_Change(VOID *audio_stream,ULONG alternate_setting);
VOID USBD_AUDIO_SetControlValues(VOID);
VOID USBD_AUDIO_Handle_SOF(VOID);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif  /* __UX_DEVICE_AUDIO_H__ */
