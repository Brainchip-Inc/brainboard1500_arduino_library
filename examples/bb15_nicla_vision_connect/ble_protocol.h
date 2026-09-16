#ifndef BB15_NICLA_VISION_CONNECT_BLE_PROTOCOL_H_
#define BB15_NICLA_VISION_CONNECT_BLE_PROTOCOL_H_

#include <Arduino.h>

#include "ble_model_transfer.h"

namespace protocol {

/** @brief The name the board advertises and the app shows. */
extern const char kDeviceName[];

/**
 * @brief Manufacturer data the app identifies the board by.
 *
 * Twelve ASCII bytes: two for the Bluetooth protocol version, three for the
 * firmware version, then the chip id. The app matches on the chip id alone and
 * displays the rest, so the versions are cosmetic but should be true.
 */
extern const uint8_t kManufacturerData[12];

/** @brief Called when the phone asks for inference to start or stop. */
using DeployHandler = void (*)(model::App app, bool running);

/** @brief Called when the phone asks for the microphone stream. */
using StreamHandler = void (*)(model::App app, bool streaming);

/** @brief Called when the phone asks the board to restart. */
using ResetHandler = void (*)();

/**
 * @brief Publish the Nordic UART service and start advertising.
 *
 * Every service must be added before this is called, because ArduinoBLE
 * builds its attribute table as services are added.
 *
 * @return True when the radio came up and advertising started.
 */
bool begin();

/**
 * @brief Register the callbacks the command handlers report through.
 *
 * @param onDeploy  Called for the start and stop inference commands.
 * @param onStream  Called for the start and stop streaming commands.
 * @param onReset   Called for the reset command.
 */
void setHandlers(DeployHandler onDeploy, StreamHandler onStream,
                 ResetHandler onReset);

/**
 * @brief Answer any command the phone has sent.
 *
 * Commands are answered here rather than in the Bluetooth write callback that
 * received them, because a single command can send six notifications.
 */
void poll();

/**
 * @brief Tell the phone about a keyword detection.
 *
 * @param label       Class label to show.
 * @param confidence  Score for that class, from zero to one.
 */
void sendDetection(const char* label, float confidence);

/**
 * @brief Send one frame of the microphone waveform.
 *
 * @param values  Interleaved minimum and maximum samples.
 * @param count   Number of values, which the phone reads as the mode.
 */
void sendWaveform(const int16_t* values, uint16_t count);

/**
 * @brief Offer one camera image for preview.
 *
 * An image takes many notifications to send, and the camera produces them
 * faster than the link carries them, so an offer made while one is still
 * going out is dropped. The phone therefore always gets whole images, each
 * the newest one available when its turn came, and the backlog never grows.
 *
 * The pixels are copied, so the caller may reuse its buffer at once.
 *
 * @param pixels  8-bit grayscale pixels, top row first.
 * @param width   Image width in pixels, at most 255.
 * @param height  Image height in pixels, at most 255.
 */
void offerPreview(const uint8_t* pixels, uint8_t width, uint8_t height);

/**
 * @brief Send a bounded slice of the image being previewed.
 *
 * Call it from the sketch's loop. It sends a few notifications and returns, so
 * that a link slow to take them delays the preview and nothing else.
 */
void pollPreview();

/** @brief Say whether a phone is connected and subscribed. */
bool connected();

}  // namespace protocol

#endif  // BB15_NICLA_VISION_CONNECT_BLE_PROTOCOL_H_
