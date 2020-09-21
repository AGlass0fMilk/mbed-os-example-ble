/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "gap/Gap.h"
#include "gap/AdvertisingDataParser.h"
#include "pretty_printer.h"
#include "power_save.h"
#include "rtos/EventFlags.h"

rtos::EventFlags flags;

/** This example demonstrates all the basic setup required
 *  to advertise, scan and connect to other devices.
 *
 *  It contains a single class that performs both scans and advertisements.
 *
 *  The demonstrations happens in sequence, after each "mode" ends
 *  the demo jumps to the next mode to continue. There are several modes
 *  that show scanning and several showing advertising. These are configured
 *  according to the two arrays containing parameters. During scanning
 *  a connection will be made to a connectable device upon its discovery.
 */

events::EventQueue event_queue;

/* Duration of each mode in milliseconds */
static const size_t MODE_DURATION_MS      = 6000;

/* Time between each mode in milliseconds */
static const size_t TIME_BETWEEN_MODES_MS = 2000;

/* how long to wait before disconnecting in milliseconds */
static const size_t CONNECTION_DURATION   = 3000;

/* how many advertising sets we want to crate at once */
static const uint8_t ADV_SET_NUMBER       = 2;

static const uint16_t MAX_ADVERTISING_PAYLOAD_SIZE = 1000;

typedef struct {
    ble::advertising_type_t type;
    ble::adv_interval_t min_interval;
    ble::adv_interval_t max_interval;
} DemoAdvParams_t;

typedef struct {
    ble::scan_interval_t interval;
    ble::scan_window_t   window;
    ble::scan_duration_t duration;
    bool active;
} DemoScanParam_t;

/** the entries in this array are used to configure our advertising
 *  parameters for each of the modes we use in our demo */
static const DemoAdvParams_t advertising_params[] = {
/*    advertising type                                   | min interval - 0.625us  | max interval - 0.625us   */
    { ble::advertising_type_t::CONNECTABLE_UNDIRECTED,      ble::adv_interval_t(40), ble::adv_interval_t(80)  },
    { ble::advertising_type_t::SCANNABLE_UNDIRECTED,       ble::adv_interval_t(100), ble::adv_interval_t(200) },
    { ble::advertising_type_t::NON_CONNECTABLE_UNDIRECTED, ble::adv_interval_t(100), ble::adv_interval_t(200) }
};

/* when we cycle through all our advertising modes we will move to scanning modes */

/** the entries in this array are used to configure our scanning
 *  parameters for each of the modes we use in our demo */
static const DemoScanParam_t scanning_params[] = {
/*                      interval                  window                   duration  active */
/*                      0.625ms                  0.625ms                       10ms         */
    {   ble::scan_interval_t(4),   ble::scan_window_t(4),   ble::scan_duration_t(0), false },
    { ble::scan_interval_t(160), ble::scan_window_t(100), ble::scan_duration_t(300), false },
    { ble::scan_interval_t(160),  ble::scan_window_t(40),   ble::scan_duration_t(0), true  },
    { ble::scan_interval_t(500),  ble::scan_window_t(10),   ble::scan_duration_t(0), false }
};

/* helper that gets the number of items in arrays */
template<class T, size_t N>
size_t arraysize(const T (&)[N])
{
    return N;
}

/** Demonstrate advertising, scanning and connecting
 */
class GapDemo : private mbed::NonCopyable<GapDemo>, public ble::Gap::EventHandler
{
public:
    GapDemo(BLE& ble, events::EventQueue& event_queue) :
        _ble(ble),
        _gap(ble.gap()),
        _event_queue(event_queue),
        _set_index(0),
        _is_in_scanning_mode(true),
        _is_connecting(false),
        _on_duration_end_id(0),
        _scan_count(0),
        _blink_event(0) {
        for (uint8_t i = 0; i < arraysize(_adv_handles); ++i) {
            _adv_handles[i] = ble::INVALID_ADVERTISING_HANDLE;
        }
    }

    ~GapDemo()
    {
        if (_ble.hasInitialized()) {
            _ble.shutdown();
        }
    }

    /** Start BLE interface initialisation */
    void run()
    {
        if (_ble.hasInitialized()) {
            printf("Ble instance already initialised.\r\n");
            return;
        }

        /* handle gap events */
        _gap.setEventHandler(this);

        ble_error_t error = _ble.init(this, &GapDemo::on_init_complete);
        if (error) {
            print_error(error, "Error returned by BLE::init");
            return;
        }

        /* this will not return until shutdown */
        while(true) {
            flags.wait_all(0b1);
            _event_queue.dispatch(10);
        }
    }

private:
    /** This is called when BLE interface is initialised and starts the first mode */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *event)
    {
        if (event->error) {
            print_error(event->error, "Error during the initialisation");
            return;
        }

        print_mac_address();

        /* setup the default phy used in connection to 2M to reduce power consumption */
        if (is_2m_phy_supported()) {
            ble::phy_set_t phys(/* 1M */ false, /* 2M */ true, /* coded */ false);

            ble_error_t error = _gap.setPreferredPhys(/* tx */&phys, /* rx */&phys);
            if (error) {
                print_error(error, "GAP::setPreferedPhys failed");
            }
        }

        /* all calls are serialised on the user thread through the event queue */
        _event_queue.call(this, &GapDemo::demo_mode_start);
    }

    /** queue up start of the current demo mode */
    void demo_mode_start()
    {
        _event_queue.call(this, &GapDemo::scan);

        printf("\r\n");
    }

    /** Set up and start scanning */
    void scan()
    {
        const DemoScanParam_t &scan_params = scanning_params[_set_index];

        /*
         * Scanning happens repeatedly and is defined by:
         *  - The scan interval which is the time (in 0.625us) between each scan cycle.
         *  - The scan window which is the scanning time (in 0.625us) during a cycle.
         * If the scanning process is active, the local device sends scan requests
         * to discovered peer to get additional data.
         */
        ble_error_t error = _gap.setScanParameters(
            ble::ScanParameters(
                ble::phy_t::LE_1M,   // scan on the 1M PHY
                ble::scan_interval_t(16000),
                ble::scan_window_t(10),
                true
            )
        );
        if (error) {
            print_error(error, "Error caused by Gap::setScanParameters");
            return;
        }

        /* start scanning and attach a callback that will handle advertisements
         * and scan requests responses */
        error = _gap.startScan(ble::scan_duration_t(0));
        if (error) {
            print_error(error, "Error caused by Gap::startScan");
            return;
        }

        printf("Scanning started (interval: %dms, window: %dms, timeout: %dms).\r\n",
               scan_params.interval.valueInMs(), scan_params.window.valueInMs(), scan_params.duration.valueInMs());
    }

    /** Execute the disconnection */
    void do_disconnect(ble::connection_handle_t handle)
    {
        printf("Disconnecting\r\n");
        _gap.disconnect(handle, ble::local_disconnection_reason_t::USER_TERMINATION);
    }

    bool is_2m_phy_supported()
    {
        return _gap.isFeatureSupported(ble::controller_supported_features_t::LE_2M_PHY);
    }

    bool is_extended_advertising_supported()
    {
        return _gap.isFeatureSupported(ble::controller_supported_features_t::LE_EXTENDED_ADVERTISING);
    }


private:
    /* Gap::EventHandler */

    /** Look at scan payload to find a peer device and connect to it */
    virtual void onAdvertisingReport(const ble::AdvertisingReportEvent &event)
    {
        /* keep track of scan events for performance reporting */
        _scan_count++;

        /* only look at events from devices at a close range */
        if (event.getRssi() < -65) {
            return;
        }

//        ble::AdvertisingDataParser adv_parser(event.getPayload());
//
//        /* parse the advertising payload, looking for a discoverable device */
//        while (adv_parser.hasNext()) {
//            ble::AdvertisingDataParser::element_t field = adv_parser.next();
//        }
    }

    virtual void onAdvertisingEnd(const ble::AdvertisingEndEvent &event)
    {
        if (event.isConnected()) {
            printf("Stopped advertising early due to connection\r\n");
        }
    }

    virtual void onScanTimeout(const ble::ScanTimeoutEvent&)
    {
        printf("Stopped scanning early due to timeout parameter\r\n");
    }

    /** This is called by Gap to notify the application we connected,
     *  in our case it immediately disconnects */
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event)
    {

    }

    /** This is called by Gap to notify the application we disconnected,
     *  in our case it calls next_demo_mode() to progress the demo */
    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event)
    {
        printf("Disconnected\r\n");
    }

    /**
     * Implementation of Gap::EventHandler::onReadPhy
     */
    virtual void onReadPhy(
        ble_error_t error,
        ble::connection_handle_t connectionHandle,
        ble::phy_t txPhy,
        ble::phy_t rxPhy
    ) {
        if (error) {
            printf(
                "Phy read on connection %d failed with error code %s\r\n",
                connectionHandle,
                BLE::errorToString(error)
            );
        } else {
            printf(
                "Phy read on connection %d - Tx Phy: %s, Rx Phy: %s\r\n",
                connectionHandle,
                phy_to_string(txPhy),
                phy_to_string(rxPhy)
            );
        }
    }

    /**
     * Implementation of Gap::EventHandler::onPhyUpdateComplete
     */
    virtual void onPhyUpdateComplete(
        ble_error_t error,
        ble::connection_handle_t connectionHandle,
        ble::phy_t txPhy,
        ble::phy_t rxPhy
    ) {
        if (error) {
            printf(
                "Phy update on connection: %d failed with error code %s\r\n",
                connectionHandle,
                BLE::errorToString(error)
            );
        } else {
            printf(
                "Phy update on connection %d - Tx Phy: %s, Rx Phy: %s\r\n",
                connectionHandle,
                phy_to_string(txPhy),
                phy_to_string(rxPhy)
            );
        }
    }

    /**
     * Implementation of Gap::EventHandler::onDataLengthChange
     */
    virtual void onDataLengthChange(
        ble::connection_handle_t connectionHandle,
        uint16_t txSize,
        uint16_t rxSize
    ) {
        printf(
            "Data length changed on the connection %d.\r\n"
            "Maximum sizes for over the air packets are:\r\n"
            "%d octets for transmit and %d octets for receive.\r\n",
            connectionHandle,
            txSize,
            rxSize
        );
    }

private:


    /** Blink LED to show we're running */
    void blink(void)
    {
//        _led1 = !_led1;
    }

private:
    BLE                &_ble;
    ble::Gap           &_gap;
    events::EventQueue &_event_queue;

    /* Keep track of our progress through demo modes */
    size_t              _set_index;
    bool                _is_in_scanning_mode;
    bool                _is_connecting;

    /* Remember the call id of the function on _event_queue
     * so we can cancel it if we need to end the mode early */
    int                 _on_duration_end_id;

    /* Measure performance of our advertising/scanning */
    size_t              _scan_count;

    int                 _blink_event;

    ble::advertising_handle_t _adv_handles[ADV_SET_NUMBER];
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
    flags.set(0b1);
}

int main()
{
    power_save();
    BLE &ble = BLE::Instance();

    /* this will inform us off all events so we can schedule their handling
     * using our event queue */
    ble.onEventsToProcess(schedule_ble_events);

    GapDemo demo(ble, event_queue);

    while (1) {
        demo.run();
        //thread_sleep_for(TIME_BETWEEN_MODES_MS);
        //printf("\r\nStarting next GAP demo mode\r\n");
    };

    return 0;
}
