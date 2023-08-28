#include "Samples.h"
#include <com/amazonaws/kinesis/video/webrtcclient/Include.h>
#include <kinesis_video_msgs/KinesisImageMetadata.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>

#include <ros/ros.h>

#include <csignal>
#include <cstdlib>
#include <cstdio>
#include <cstring>

extern PSampleConfiguration gSampleConfiguration;

STATUS signalingCallFailed(STATUS status)
{
    return (STATUS_SIGNALING_GET_TOKEN_CALL_FAILED == status || STATUS_SIGNALING_DESCRIBE_CALL_FAILED == status ||
            STATUS_SIGNALING_CREATE_CALL_FAILED == status || STATUS_SIGNALING_GET_ENDPOINT_CALL_FAILED == status ||
            STATUS_SIGNALING_GET_ICE_CONFIG_CALL_FAILED == status || STATUS_SIGNALING_CONNECT_CALL_FAILED == status);
}

void videoFrameCallback(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr& kvs_image)
{
    STATUS retStatus = STATUS_SUCCESS;
    if (gSampleConfiguration == NULL) {
        printf("[KVS Master] sendVideoPackets(): operation returned status code: 0x%08x \n", STATUS_NULL_ARG);
        return;
    }
    RtcEncoderStats encoderStats;
    UINT32 fileIndex = 0, frameSize;
    CHAR filePath[MAX_PATH_LEN + 1];
    STATUS status;
    UINT32 i;
    UINT64 startTime, lastFrameTime, elapsed;
    Frame frame;
    MEMSET(&encoderStats, 0x00, SIZEOF(RtcEncoderStats));
    frameSize = kvs_image->frame_data.size() + kvs_image->codec_private_data.size();
    gSampleConfiguration->pVideoFrameBuffer = (PBYTE) realloc(gSampleConfiguration->pVideoFrameBuffer, frameSize);
    if (gSampleConfiguration->pVideoFrameBuffer == NULL) {
        printf("[KVS Master] Video frame Buffer reallocation failed...%s (code %d)\n", strerror(errno), errno);
        printf("[KVS Master] realloc(): operation returned status code: 0x%08x \n", STATUS_NOT_ENOUGH_MEMORY);
    }
    gSampleConfiguration->videoBufferSize = frameSize;

    frame.frameData = gSampleConfiguration->pVideoFrameBuffer;
    std::vector<uint8_t> h264_data = kvs_image->codec_private_data;
    h264_data.insert(h264_data.end(), kvs_image->frame_data.begin(), kvs_image->frame_data.end());
    frame.frameData = reinterpret_cast<PBYTE>((void*) (h264_data.data()));
    //    BYTE D = BYTE(10);
    //    frame.frameData = &D;
    frame.size = frameSize;
    frame.flags = (FRAME_FLAGS) kvs_image->flags;
    frame.index = kvs_image->index;

    //    retStatus = readFrameFromDisk(frame.frameData, &frameSize, filePath);
    // if(retStatus != STATUS_SUCCESS) {
    //   printf("[KVS Master] readFrameFromDisk(): operation returned status code: 0x%08x \n", retStatus);
    //   //goto CleanUp;
    // }
    encoderStats.width = 640;
    encoderStats.height = 480;
    encoderStats.targetBitrate = 262000;
    frame.duration = kvs_image->duration;
    frame.decodingTs = kvs_image->decoding_ts;
    frame.presentationTs = kvs_image->presentation_ts;
    for (i = 0; i < gSampleConfiguration->streamingSessionCount; ++i) {
        status = writeFrame(gSampleConfiguration->sampleStreamingSessionList[i]->pVideoRtcRtpTransceiver, &frame);
        encoderStats.encodeTimeMsec = 4; // update encode time to an arbitrary number to demonstrate stats update
        updateEncoderStats(gSampleConfiguration->sampleStreamingSessionList[i]->pVideoRtcRtpTransceiver, &encoderStats);
        if (status != STATUS_SRTP_NOT_READY_YET) {
            if (status != STATUS_SUCCESS) {
                printf("writeFrame() failed with 0x%08x\n", status);
            }
        }
    }
}
void init(int argc, char* argv[])
{
    STATUS retStatus = STATUS_SUCCESS;
    UINT32 frameSize;
    PSampleConfiguration pSampleConfiguration = NULL;
    SignalingClientMetrics signalingClientMetrics;
    std::string pChannelName;
    signalingClientMetrics.version = SIGNALING_CLIENT_METRICS_CURRENT_VERSION;

    SET_INSTRUMENTED_ALLOCATORS();

#ifndef _WIN32
    signal(SIGINT, sigintHandler);
#endif

    // do trickleIce by default
    std::cout << "[KVS Master] Using trickleICE by default\n";

#ifdef IOT_CORE_ENABLE_CREDENTIALS
    if (!(pChannelName = std::getenv(IOT_CORE_THING_NAME))) {
        std::cerr << "AWS_IOT_CORE_THING_NAME must be set";
        exit(STATUS_INVALID_OPERATION);
    }
#else
    pChannelName = argc > 1 ? argv[1] : SAMPLE_CHANNEL_NAME;
#endif

    char* mutableCString = strdup(pChannelName.c_str());
    if (mutableCString == NULL) {
        std::cerr << "Memory allocation for mutableCString failed.";
        exit(STATUS_INVALID_OPERATION);
    }

    retStatus = createSampleConfiguration(mutableCString, SIGNALING_CHANNEL_ROLE_TYPE_MASTER, TRUE, TRUE, &pSampleConfiguration);
    // free(mutableCString); // don't forget to free the memory after usage
    if (retStatus != STATUS_SUCCESS) {
        std::cout << "[KVS Master] createSampleConfiguration(): operation returned status code: " << std::hex << retStatus << "\n";
        // TODO goto CleanUp;
    }

    std::cout << "[KVS Master] Created signaling channel " << pChannelName << "\n";

    if (pSampleConfiguration->enableFileLogging) {
        retStatus =
            createFileLogger(FILE_LOGGING_BUFFER_SIZE, MAX_NUMBER_OF_LOG_FILES, (PCHAR) FILE_LOGGER_LOG_FILE_DIRECTORY_PATH, TRUE, TRUE, NULL);
        if (retStatus != STATUS_SUCCESS) {
            std::cout << "[KVS Master] createFileLogger(): operation returned status code: " << std::hex << retStatus << "\n";
            pSampleConfiguration->enableFileLogging = FALSE;
        }
    }

    // Set the audio and video handlers
    pSampleConfiguration->onDataChannel = onDataChannel;
    pSampleConfiguration->mediaType = SAMPLE_STREAMING_AUDIO_VIDEO;
    std::cout << "[KVS Master] Finished setting video handlers\n";

    // Check if the samples are present

    // Initialize KVS WebRTC. This must be done before anything else, and must only be done once.
    retStatus = initKvsWebRtc();
    if (retStatus != STATUS_SUCCESS) {
        std::cout << "[KVS Master] initKvsWebRtc(): operation returned status code: " << std::hex << retStatus << "\n";
        // // goto CleanUp;
    }
    std::cout << "[KVS Master] KVS WebRTC initialization completed successfully\n";

    pSampleConfiguration->signalingClientCallbacks.messageReceivedFn = signalingMessageReceived;

    strcpy(pSampleConfiguration->clientInfo.clientId, SAMPLE_MASTER_CLIENT_ID);

    retStatus = createSignalingClientSync(&pSampleConfiguration->clientInfo, &pSampleConfiguration->channelInfo,
                                          &pSampleConfiguration->signalingClientCallbacks, pSampleConfiguration->pCredentialProvider,
                                          &pSampleConfiguration->signalingClientHandle);
    if (retStatus != STATUS_SUCCESS) {
        std::cout << "[KVS Master] createSignalingClientSync(): operation returned status code: " << std::hex << retStatus << "\n";
        // goto CleanUp;
    }
    std::cout << "[KVS Master] Signaling client created successfully\n";

    retStatus = signalingClientFetchSync(pSampleConfiguration->signalingClientHandle);
    if (retStatus != STATUS_SUCCESS) {
        printf("[KVS Master] signalingClientFetchSync(): operation returned status code: 0x%08x \n", retStatus);
        // goto CleanUp;
    }

    retStatus = signalingClientConnectSync(pSampleConfiguration->signalingClientHandle);
    if (retStatus != STATUS_SUCCESS) {
        printf("[KVS Master] signalingClientConnectSync(): operation returned status code: 0x%08x \n", retStatus);
        // goto CleanUp;
    }
    printf("[KVS Master] Signaling client connection to socket established\n");

    gSampleConfiguration = pSampleConfiguration;
}

void sessionCleanup(PSampleConfiguration pSampleConfiguration)
{
    ENTERS();
    STATUS retStatus = STATUS_SUCCESS;
    PSampleStreamingSession pSampleStreamingSession = NULL;
    UINT32 i, clientIdHash;
    BOOL locked = FALSE, peerConnectionFound = FALSE;
    SIGNALING_CLIENT_STATE signalingClientState;
    CHK(pSampleConfiguration != NULL, STATUS_NULL_ARG);

    // scan and cleanup terminated streaming session
    for (i = 0; i < pSampleConfiguration->streamingSessionCount; ++i) {
        if (ATOMIC_LOAD_BOOL(&pSampleConfiguration->sampleStreamingSessionList[i]->terminateFlag)) {
            pSampleStreamingSession = pSampleConfiguration->sampleStreamingSessionList[i];

            // swap with last element and decrement count
            pSampleConfiguration->streamingSessionCount--;
            pSampleConfiguration->sampleStreamingSessionList[i] =
                pSampleConfiguration->sampleStreamingSessionList[pSampleConfiguration->streamingSessionCount];

            // Remove from the hash table
            clientIdHash = COMPUTE_CRC32((PBYTE) pSampleStreamingSession->peerId, (UINT32) STRLEN(pSampleStreamingSession->peerId));
            CHK_STATUS(hashTableContains(pSampleConfiguration->pRtcPeerConnectionForRemoteClient, clientIdHash, &peerConnectionFound));
            if (peerConnectionFound) {
                CHK_STATUS(hashTableRemove(pSampleConfiguration->pRtcPeerConnectionForRemoteClient, clientIdHash));
            }
            CHK_STATUS(freeSampleStreamingSession(&pSampleStreamingSession));
        }
    }

    // Check if we need to re-create the signaling client on-the-fly
    if (ATOMIC_LOAD_BOOL(&pSampleConfiguration->recreateSignalingClient)) {
        retStatus = signalingClientFetchSync(pSampleConfiguration->signalingClientHandle);
        if (STATUS_SUCCEEDED(retStatus)) {
            // Re-set the variable again
            ATOMIC_STORE_BOOL(&pSampleConfiguration->recreateSignalingClient, FALSE);
        }
        else if (signalingCallFailed(retStatus)) {
            printf("[KVS Common] recreating Signaling Client\n");
            freeSignalingClient(&pSampleConfiguration->signalingClientHandle);
            createSignalingClientSync(&pSampleConfiguration->clientInfo, &pSampleConfiguration->channelInfo,
                                        &pSampleConfiguration->signalingClientCallbacks, pSampleConfiguration->pCredentialProvider,
                                        &pSampleConfiguration->signalingClientHandle);
        }
    }

    // Check the signaling client state and connect if needed
    if (IS_VALID_SIGNALING_CLIENT_HANDLE(pSampleConfiguration->signalingClientHandle)) {
        CHK_STATUS(signalingClientGetCurrentState(pSampleConfiguration->signalingClientHandle, &signalingClientState));
        if (signalingClientState == SIGNALING_CLIENT_STATE_READY) {
            UNUSED_PARAM(signalingClientConnectSync(pSampleConfiguration->signalingClientHandle));
        }
    }

    // Check if any lingering pending message queues
    CHK_STATUS(removeExpiredMessageQueues(pSampleConfiguration->pPendingSignalingMessageForRemoteClient));

    // periodically wake up and clean up terminated streaming session
    CVAR_WAIT(pSampleConfiguration->cvar, pSampleConfiguration->sampleConfigurationObjLock, SAMPLE_SESSION_CLEANUP_WAIT_PERIOD);
    locked = FALSE;
CleanUp:

    CHK_LOG_ERR(retStatus);

    LEAVES();
}

int main(int argc, char* argv[])
{
    init(argc, argv);
    std::string inputTopicName = argc > 2 ? argv[2] : SAMPLE_INPUT_TOPIC_NAME;
    ros::init(argc, argv, "kinesis_vide_webrtc");
    ros::NodeHandle n;
    ros::Subscriber something = n.subscribe(inputTopicName, 10, videoFrameCallback);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        sessionCleanup(gSampleConfiguration);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
