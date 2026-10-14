/* CDC ACM + UAC1 adaptive mono speaker on the existing ST single-class core.
 * Keep pClassData/pUserData owned by CDC: all existing CDC public APIs and
 * completion callbacks continue to address the same handle without swaps. */
#include "usbd_cdc_audio.h"
#include "usbd_cdc.h"
#include "usbd_ctlreq.h"
#include "usb_audio.h"
#include <string.h>

#define UAC_FEATURE_ID 2u
#define UAC_SET_CUR 0x01u
#define UAC_GET_CUR 0x81u
#define UAC_GET_MIN 0x82u
#define UAC_GET_MAX 0x83u
#define UAC_GET_RES 0x84u
#define UAC_MUTE_CONTROL 1u
#define UAC_VOLUME_CONTROL 2u
#define UAC_RATE_CONTROL 1u

enum { EP0_NONE, EP0_CDC, EP0_MUTE, EP0_VOLUME, EP0_RATE };
static uint8_t ep0_owner, alternate, suspended;
static uint8_t control[4] __attribute__((aligned(4)));
/* RX PMA rounds a 98-byte endpoint to 128 bytes.  Keep the RAM staging buffer
 * that size too: a malformed host packet cannot overflow it before validation. */
static uint8_t audio_rx[UAC_PMA_BYTES] __attribute__((aligned(4)));

/* ST core writes descriptor[1] when serving GET_CONFIGURATION, hence this
 * one small configuration descriptor is writable.  All FIR tables are const. */
static uint8_t configuration[UAC_CONFIG_DESC_SIZE] __attribute__((aligned(4))) = {
  9, USB_DESC_TYPE_CONFIGURATION, UAC_CONFIG_DESC_SIZE, 0, 4, 1, 0,
#if USBD_SELF_POWERED == 1u
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,
  /* CDC IAD, keep original interfaces 0/1 and endpoints 81/01/82. */
  8, 0x0B, 0, 2, 0x02, 0x02, 0x01, 0,
  9, 0x04, 0, 0, 1, 0x02, 0x02, 0x01, 0,
  5, 0x24, 0x00, 0x10, 0x01,
  5, 0x24, 0x01, 0x00, 1,
  4, 0x24, 0x02, 0x02,
  5, 0x24, 0x06, 0, 1,
  7, 0x05, CDC_CMD_EP, 0x03, CDC_CMD_PACKET_SIZE, 0, CDC_FS_BINTERVAL,
  9, 0x04, 1, 0, 2, 0x0A, 0, 0, 0,
  7, 0x05, CDC_OUT_EP, 0x02, CDC_DATA_FS_MAX_PACKET_SIZE, 0, 0,
  7, 0x05, CDC_IN_EP, 0x02, CDC_DATA_FS_MAX_PACKET_SIZE, 0, 0,
  /* Audio IAD, AudioControl interface 2, AudioStreaming interface 3. */
  8, 0x0B, UAC_CONTROL_INTERFACE, 2, 0x01, 0x00, 0x00, USBD_IDX_INTERFACE_STR,
  9, 0x04, UAC_CONTROL_INTERFACE, 0, 0, 0x01, 0x01, 0, USBD_IDX_INTERFACE_STR,
  /* UAC1 AC header; class-specific total = 9 + 12 + 9 + 9 = 39. */
  9, 0x24, 0x01, 0x00, 0x01, 39, 0, 1, UAC_STREAM_INTERFACE,
  /* USB streaming input terminal ID1, one channel with no spatial location. */
  12, 0x24, 0x02, 1, 0x01, 0x01, 0, 1, 0, 0, 0, 0,
  /* Feature ID2: master mute+volume; channel 1 has no separate controls. */
  9, 0x24, 0x06, UAC_FEATURE_ID, 1, 1, 0x03, 0x00, 0,
  /* Speaker output terminal ID3, source feature ID2. */
  9, 0x24, 0x03, 3, 0x01, 0x03, 0, UAC_FEATURE_ID, 0,
  /* AS alt0 has no endpoint, alt1 activates PCM playback. */
  9, 0x04, UAC_STREAM_INTERFACE, 0, 0, 0x01, 0x02, 0, 0,
  9, 0x04, UAC_STREAM_INTERFACE, 1, 1, 0x01, 0x02, 0, 0,
  7, 0x24, 0x01, 1, 1, 0x01, 0x00, /* terminal 1, delay 1, PCM */
  11, 0x24, 0x02, 1, 1, 2, 16, 1, 0x80, 0xBB, 0x00, /* 48000 Hz */
  /* Adaptive isochronous OUT: output clock recovery is in aperture_mod.
   * No feedback endpoint or SOF interrupt is needed for an adaptive sink. */
  9, 0x05, UAC_OUT_EP, 0x09, UMH_USB_AUDIO_PACKET_MAX, 0, 1, 0, 0,
  7, 0x25, 0x01, 0x01, 0, 0, 0 /* fixed sampling-frequency control */
};
_Static_assert(sizeof(configuration) == UAC_CONFIG_DESC_SIZE, "composite descriptor size");
_Static_assert(UAC_PMA_OUT >= 0x148u && UAC_PMA_OUT + UAC_PMA_BYTES <= 512u,
               "audio PMA must not overlap CDC and must fit even a 512-byte PMA");

static uint8_t *get_config(uint16_t *length)
{
  *length = sizeof(configuration);
  return configuration;
}

static uint8_t fail(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_CtlError(pdev, req);
  return USBD_FAIL;
}

static void close_audio(USBD_HandleTypeDef *pdev)
{
  umh_usb_audio_set_stream(0u);
  if (alternate != 0u) {
    (void)USBD_LL_CloseEP(pdev, UAC_OUT_EP);
    pdev->ep_out[UAC_OUT_EP].is_used = 0u;
  }
  alternate = 0u;
}

static uint8_t composite_init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  alternate = suspended = ep0_owner = 0u;
  umh_usb_audio_init();
  return USBD_CDC.Init(pdev, cfgidx);
}

static uint8_t composite_deinit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  close_audio(pdev);
  ep0_owner = EP0_NONE;
  return USBD_CDC.DeInit(pdev, cfgidx);
}

static uint8_t audio_standard(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t iface = (uint8_t)req->wIndex;
  if (pdev->dev_state != USBD_STATE_CONFIGURED || req->wIndex != iface)
    return fail(pdev, req);
  switch (req->bRequest) {
    case USB_REQ_GET_STATUS:
      if (req->bmRequest != 0x81u || req->wLength != 2u || req->wValue != 0u)
        return fail(pdev, req);
      control[0] = control[1] = 0u;
      return USBD_CtlSendData(pdev, control, 2u);
    case USB_REQ_GET_INTERFACE:
      if (req->bmRequest != 0x81u || req->wLength != 1u || req->wValue != 0u)
        return fail(pdev, req);
      control[0] = iface == UAC_STREAM_INTERFACE ? alternate : 0u;
      return USBD_CtlSendData(pdev, control, 1u);
    case USB_REQ_SET_INTERFACE:
      if (req->bmRequest != 0x01u || req->wLength != 0u || req->wValue > 1u ||
          (iface == UAC_CONTROL_INTERFACE && req->wValue != 0u))
        return fail(pdev, req);
      if (iface == UAC_CONTROL_INTERFACE) return USBD_OK;
      close_audio(pdev);
      suspended = 0u;
      if (req->wValue == 1u) {
        if (USBD_LL_OpenEP(pdev, UAC_OUT_EP, USBD_EP_TYPE_ISOC,
                            UMH_USB_AUDIO_PACKET_MAX) != USBD_OK)
          return fail(pdev, req);
        pdev->ep_out[UAC_OUT_EP].is_used = 1u;
        alternate = 1u;
        umh_usb_audio_set_stream(1u);
        if (USBD_LL_PrepareReceive(pdev, UAC_OUT_EP, audio_rx,
                                   UMH_USB_AUDIO_PACKET_MAX) != USBD_OK) {
          close_audio(pdev);
          return fail(pdev, req);
        }
      }
      return USBD_OK;
    default:
      return fail(pdev, req);
  }
}

static uint8_t composite_setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t recipient = req->bmRequest & USB_REQ_RECIPIENT_MASK;
  uint8_t iface = (uint8_t)req->wIndex;
  uint8_t selector = (uint8_t)(req->wValue >> 8);
  uint16_t size;
  int16_t value;
  ep0_owner = EP0_NONE; /* each SETUP cancels the previous EP0 data stage */
  if (recipient == USB_REQ_RECIPIENT_INTERFACE && iface < 2u) {
    /* Old CDC class trusts wLength.  Bound it before dispatch, including the
     * SET_LINE_CODING receive, so another function cannot corrupt its state. */
    if ((req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_CLASS &&
        (req->wLength > CDC_REQ_MAX_DATA_SIZE || (req->wIndex >> 8) != 0u))
      return fail(pdev, req);
    if ((req->bmRequest & 0x80u) == 0u && req->wLength != 0u) ep0_owner = EP0_CDC;
    return USBD_CDC.Setup(pdev, req);
  }
  if (recipient == USB_REQ_RECIPIENT_ENDPOINT &&
      (iface == CDC_IN_EP || iface == CDC_OUT_EP || iface == CDC_CMD_EP))
    return USBD_CDC.Setup(pdev, req);
  if (recipient == USB_REQ_RECIPIENT_INTERFACE &&
      (iface == UAC_CONTROL_INTERFACE || iface == UAC_STREAM_INTERFACE) &&
      (req->bmRequest & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    return audio_standard(pdev, req);
  if ((req->bmRequest & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_CLASS ||
      pdev->dev_state != USBD_STATE_CONFIGURED)
    return fail(pdev, req);
  if (recipient == USB_REQ_RECIPIENT_ENDPOINT && req->wIndex == UAC_OUT_EP &&
      req->wValue == (UAC_RATE_CONTROL << 8) && req->wLength == 3u) {
    if (req->bmRequest == 0x22u && req->bRequest == UAC_SET_CUR) {
      ep0_owner = EP0_RATE;
      return USBD_CtlPrepareRx(pdev, control, 3u);
    }
    if (req->bmRequest != 0xA2u || (req->bRequest != UAC_GET_CUR &&
        req->bRequest != UAC_GET_MIN && req->bRequest != UAC_GET_MAX &&
        req->bRequest != UAC_GET_RES)) return fail(pdev, req);
    control[0] = req->bRequest == UAC_GET_RES ? 0u : 0x80u;
    control[1] = req->bRequest == UAC_GET_RES ? 0u : 0xBBu;
    control[2] = 0u;
    return USBD_CtlSendData(pdev, control, 3u);
  }
  if (recipient != USB_REQ_RECIPIENT_INTERFACE ||
      req->wIndex != ((UAC_FEATURE_ID << 8) | UAC_CONTROL_INTERFACE) ||
      (req->wValue & 0xFFu) != 0u ||
      (selector != UAC_MUTE_CONTROL && selector != UAC_VOLUME_CONTROL))
    return fail(pdev, req);
  size = selector == UAC_MUTE_CONTROL ? 1u : 2u;
  if (req->wLength != size) return fail(pdev, req);
  if (req->bmRequest == 0x21u && req->bRequest == UAC_SET_CUR) {
    ep0_owner = selector == UAC_MUTE_CONTROL ? EP0_MUTE : EP0_VOLUME;
    return USBD_CtlPrepareRx(pdev, control, size);
  }
  if (req->bmRequest != 0xA1u) return fail(pdev, req);
  if (selector == UAC_MUTE_CONTROL) {
    if (req->bRequest != UAC_GET_CUR) return fail(pdev, req);
    control[0] = umh_usb_audio_mute();
  } else {
    switch (req->bRequest) {
      case UAC_GET_CUR: value = umh_usb_audio_volume(); break;
      case UAC_GET_MIN: value = -60 * 256; break;
      case UAC_GET_MAX: value = 0; break;
      case UAC_GET_RES: value = 256; break;
      default: return fail(pdev, req);
    }
    control[0] = (uint8_t)value;
    control[1] = (uint8_t)((uint16_t)value >> 8);
  }
  return USBD_CtlSendData(pdev, control, size);
}

static uint8_t ep0_received(USBD_HandleTypeDef *pdev)
{
  uint8_t owner = ep0_owner;
  ep0_owner = EP0_NONE;
  switch (owner) {
    case EP0_CDC: return USBD_CDC.EP0_RxReady(pdev);
    case EP0_MUTE:
      if (control[0] > 1u) return fail(pdev, &pdev->request);
      umh_usb_audio_set_mute(control[0]);
      break;
    case EP0_VOLUME:
      umh_usb_audio_set_volume((int16_t)((uint16_t)control[0] | ((uint16_t)control[1] << 8)));
      break;
    case EP0_RATE:
      if (control[0] != 0x80u || control[1] != 0xBBu || control[2] != 0u)
        return fail(pdev, &pdev->request);
      break; /* fixed rate, do not reset the streaming epoch on this request */
    default: break;
  }
  return USBD_OK;
}

static uint8_t data_in(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  /* Interrupt notification completion must never finish a CDC bulk transfer. */
  if (epnum == (CDC_IN_EP & 0x0Fu)) return USBD_CDC.DataIn(pdev, epnum);
  return USBD_OK;
}

static uint8_t data_out(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == CDC_OUT_EP) return USBD_CDC.DataOut(pdev, epnum);
  if (epnum == UAC_OUT_EP && alternate != 0u) {
    umh_usb_audio_receive(audio_rx, (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum));
    return USBD_LL_PrepareReceive(pdev, UAC_OUT_EP, audio_rx, UMH_USB_AUDIO_PACKET_MAX);
  }
  return USBD_OK;
}

static uint8_t iso_out_incomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == UAC_OUT_EP && alternate != 0u)
    return USBD_LL_PrepareReceive(pdev, UAC_OUT_EP, audio_rx, UMH_USB_AUDIO_PACKET_MAX);
  return USBD_OK;
}

void USBD_CDC_AUDIO_Suspend(void)
{
  suspended = 1u;
  umh_usb_audio_set_stream(0u);
}

void USBD_CDC_AUDIO_Resume(void)
{
  if (suspended != 0u && alternate != 0u) umh_usb_audio_set_stream(1u);
  suspended = 0u;
}

USBD_ClassTypeDef USBD_CDC_AUDIO = {
  composite_init, composite_deinit, composite_setup, NULL, ep0_received,
  data_in, data_out, NULL, NULL, iso_out_incomplete,
  get_config, get_config, get_config, NULL,
#if USBD_SUPPORT_USER_STRING_DESC == 1u
  NULL,
#endif
};
