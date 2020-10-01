#include "driver/HMD.hpp"
#include <iostream>

using namespace northstar::driver::settings::keys;
using namespace northstar::driver::settings::values;

northstar::driver::CHMD::CHMD(
    vr::IVRSettings* pVRSettings, 
    vr::IVRServerDriverHost* pVRServerDriverHost,
    std::shared_ptr<northstar::utility::IHostProber> pHostProber,
    std::shared_ptr<northstar::openvr::IVRProperties> pVRProperties,
    std::shared_ptr<northstar::driver::IEnvironmentSensor> pEnvironmentSensor, 
    std::shared_ptr<northstar::math::IVectorFactory> pVectorFactory,
    std::shared_ptr<northstar::driver::IOptics> pOptics,
    std::shared_ptr<northstar::driver::ISensorFrameCoordinator> pSensorFrameCoordinator,
    std::shared_ptr<northstar::utility::ILogger> pLogger) {
    m_pVRSettings = pVRSettings;
    m_pVRServerDriverHost = pVRServerDriverHost;
    m_pHostProber = pHostProber;
    m_pVRProperties = pVRProperties;
    m_pLogger = pLogger;
    m_pEnvironmentSensor = pEnvironmentSensor;
    m_pVectorFactory = pVectorFactory;
    m_pOptics = pOptics;
    m_pSensorFrameCoordinator = pSensorFrameCoordinator;
    m_sOpenVRState.unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    m_sOpenVRState.ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

    LoadConfiguration();
}

void northstar::driver::CHMD::LoadConfiguration() {
    // TODO: use vector factory
    m_sConfiguration.bUseFakeScreenConfig = m_pVRSettings->GetBool(debug::k_svRoot.data(), debug::k_svUseFakeScreenConfig.data());
    m_sConfiguration.bUseFakeProjection = m_pVRSettings->GetBool(debug::k_svRoot.data(), debug::k_svUseFakeProjection.data());
    m_sConfiguration.bUseFakeWarp = m_pVRSettings->GetBool(debug::k_svRoot.data(), debug::k_svUseFakeWarp.data());
    m_sConfiguration.bUseFakeTracking = m_pVRSettings->GetBool(debug::k_svRoot.data(), debug::k_svUseFakeTracking.data());
    m_sConfiguration.dIPD = m_pVRSettings->GetFloat(display::k_svRoot.data(), display::k_svIPD.data());
    m_sConfiguration.sDisplayConfiguration.dFrequency = m_pVRSettings->GetFloat(display::k_svRoot.data(), display::k_svFrequency.data());
    m_sConfiguration.sDisplayConfiguration.dPhotonLatency = m_pVRSettings->GetFloat(display::k_svRoot.data(), display::k_svPhotonLatency.data());
    if (m_sConfiguration.bUseFakeScreenConfig) {
        //TODO: Put these in constants
        m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin << 100, 100;
        m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions << 1000, 1000;
        m_sConfiguration.sDisplayConfiguration.v2iEyeRenderAreaDimensions << 1000, 1000;
    } else {
        m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin <<
            m_pVRSettings->GetInt32(display::k_svRoot.data(), display::k_svOriginX.data()),
            m_pVRSettings->GetInt32(display::k_svRoot.data(), display::k_svOriginY.data());

        m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions <<
            m_pVRSettings->GetInt32(display::k_svRoot.data(), display::k_svWidth.data()),
            m_pVRSettings->GetInt32(display::k_svRoot.data(), display::k_svHeight.data());

        m_sConfiguration.sDisplayConfiguration.v2iEyeRenderAreaDimensions <<
            m_pVRSettings->GetInt32(display::k_svRoot.data(), display::k_svRenderWidth.data()),
            m_pVRSettings->GetInt32(display::k_svRoot.data(), display::k_svRenderHeight.data());

        if (m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin.x() < 0) {
            m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin.x() = 0;
            m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin.x() = m_pHostProber->ProbeDisplayOriginX()
                .value_or(x_iFallbackWindowOriginX);
        }
    }

    SetOpenVRConfiguration();
}

void northstar::driver::CHMD::SetOpenVRConfiguration() {
    m_pVRSettings->SetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_IPD_Float, static_cast<float>(m_sConfiguration.dIPD));
}

vr::EVRInitError northstar::driver::CHMD::Activate(vr::TrackedDeviceIndex_t unObjectId) {
    m_sOpenVRState.unObjectId = unObjectId;
    m_sOpenVRState.ulPropertyContainer = m_pVRProperties->TrackedDeviceToPropertyContainer(unObjectId);
    SetOpenVRProperties();
    return vr::VRInitError_None;
}

void northstar::driver::CHMD::SetOpenVRProperties() {
    if (m_sConfiguration.bUseFakeScreenConfig)
        m_pVRProperties->SetBoolProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);

    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_SerialNumber_String, x_svSerialNumber.data() );
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_ModelNumber_String, x_svModelNumber.data() );
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_RenderModelName_String, x_svModelNumber.data() );
    m_pVRProperties->SetFloatProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_UserIpdMeters_Float, static_cast<float>(m_sConfiguration.dIPD));
    m_pVRProperties->SetBoolProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, x_bDirectModeEnabled);
    m_pVRProperties->SetFloatProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_DisplayFrequency_Float, static_cast<float>(m_sConfiguration.sDisplayConfiguration.dFrequency));
    m_pVRProperties->SetFloatProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, static_cast<float>(m_sConfiguration.sDisplayConfiguration.dPhotonLatency));
    m_pVRProperties->SetFloatProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, static_cast<float>(x_fUserHeadToEyeDepthInMeters));
    m_pVRProperties->SetUint64Property(m_sOpenVRState.ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, driverConfiguration::k_uiCurrentUniverseID );
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, icon::k_svDeviceOff.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, icon::k_svDeviceSearching.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, icon::k_svDeviceSearchingAlert.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, icon::k_svDeviceReady.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, icon::k_svDeviceReadyAlert.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, icon::k_svDeviceNotReady.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, icon::k_svDeviceStandby.data());
    m_pVRProperties->SetStringProperty(m_sOpenVRState.ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, icon::k_svDeviceAlertLow.data());
}

void northstar::driver::CHMD::Deactivate() { 
    m_sOpenVRState.unObjectId = vr::k_unTrackedDeviceIndexInvalid; 
}

void northstar::driver::CHMD::EnterStandby() {}

void* northstar::driver::CHMD::GetComponent( const char* pchComponentNameAndVersion )
{
    if (!_stricmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version))
        return static_cast<vr::IVRDisplayComponent*>(this);

    return nullptr;
}

bool northstar::driver::CHMD::IsDisplayOnDesktop() { 
    return x_bIsDisplayOnDesktop;
}

bool northstar::driver::CHMD::IsDisplayRealDisplay() { 
    if (m_sConfiguration.bUseFakeScreenConfig)
        return false;

    return x_bIsDisplayRealDisplay; 
}


// TODO: clean up sensor driver interaction
vr::DriverPose_t northstar::driver::CHMD::GetPose() {
    static northstar::driver::IEnvironmentSensor::EPoseRetrievalError eError;

    if (m_sConfiguration.bUseFakeTracking) {
        vr::DriverPose_t sFakePose = { 0 };
        sFakePose.poseIsValid = true;
        sFakePose.deviceIsConnected = true;
        sFakePose.result = vr::TrackingResult_Running_OK;
        sFakePose.qRotation = { 1, 0, 0, 0 };
        sFakePose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
        sFakePose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
        m_pSensorFrameCoordinator->SubmitOpenVRHeadsetPose(sFakePose);
        return sFakePose;
    }

    vr::DriverPose_t Pose;
    Pose.poseIsValid = m_pEnvironmentSensor->GetPose(Pose, eError);
    if (!Pose.poseIsValid) {
        Pose.result = vr::TrackingResult_Uninitialized;
        Pose.deviceIsConnected = false;
        return Pose;
    }

    std::string debug = "";
    debug += "%%%%%% Position: (" + std::to_string(Pose.vecPosition[0]) + ", " + std::to_string(Pose.vecPosition[1]) + ", " + std::to_string(Pose.vecPosition[2]) + ")\n";
    m_pLogger->Log(debug.data());

    Pose.result = vr::TrackingResult_Running_OK;
    Pose.deviceIsConnected = true;
    m_pSensorFrameCoordinator->SubmitOpenVRHeadsetPose(Pose);
    return Pose;
}

void northstar::driver::CHMD::GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) {
    *pnX = m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin.x();
    *pnY = m_sConfiguration.sDisplayConfiguration.v2iWindowOrigin.y();
    *pnWidth = m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions.x();
    *pnHeight = m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions.y();
}

void northstar::driver::CHMD::GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) {
    *pnWidth = m_sConfiguration.sDisplayConfiguration.v2iEyeRenderAreaDimensions.x();
    *pnHeight = m_sConfiguration.sDisplayConfiguration.v2iEyeRenderAreaDimensions.y();
}

void northstar::driver::CHMD::GetEyeOutputViewport(vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) {
    *pnY = 0;
    *pnWidth = m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions.x() / 2;
    *pnHeight = m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions.y();
    if (eEye == vr::Eye_Left)
        *pnX = 0;
    else
        *pnX = m_sConfiguration.sDisplayConfiguration.v2iWindowDimensions.x() / 2;
}

void northstar::driver::CHMD::GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) {
    if (m_sConfiguration.bUseFakeProjection) {
        *pfLeft = -1.0;
        *pfRight = 1.0;
        *pfTop = -1.0;
        *pfBottom = 1.0;
        return;
    }

    auto v4dEyeProjectionLRTB = m_pOptics->GetEyeProjectionLRTB(eEye);
    *pfLeft = static_cast<float>(v4dEyeProjectionLRTB.x());
    *pfRight = static_cast<float>(v4dEyeProjectionLRTB.y());
    *pfTop = static_cast<float>(v4dEyeProjectionLRTB.z());
    *pfBottom = static_cast<float>(v4dEyeProjectionLRTB.w());	
}

float left_uv_to_rect_x[] = {-0.03025273194246765, -10.332571869617775, 22.920310193688138, -16.245127334079367, -6.4111247104759, 74.87936605663893, -152.32992421214445, 100.07790752961323, 18.767075862784353, -159.6456296535261, 314.0055045039939, -199.7048205422684, -12.222839842933524, 103.4463706973384, -203.21519558449378, 129.17007049482532};
float left_uv_to_rect_y[] = {0.029115293434358536, -5.218962153638652, 14.777386870023445, -8.394642473641362, -9.493554619201916, 67.53183793641266, -140.95499249383968, 87.00082242941332, 23.127112207617238, -153.11976700720265, 309.5947997732527, -191.62131264722873, -17.1336637578178, 107.63373033396942, -211.82844939981243, 131.12335568460153};
float right_uv_to_rect_x[] = {0.15757501376143696, 8.707879246106318, -19.470374384585757, 13.57135190184141, 4.528897834236504, -62.08850498066993, 126.63715545848075, -82.01165869765885, -13.797466335165645, 128.79708468105366, -252.8309813610479, 158.34515911279317, 7.982020511442282, -79.08045643764393, 155.7654807989786, -97.92641609725516};
float right_uv_to_rect_y[] =  {-0.04966280748846229, 6.194050807470437, -16.74078406698002, 9.942203858070798, 10.912716733843816, -73.7042758899008, 147.43414219879185, -89.54141901996026, -23.808111295373465, 148.6614948167901, -287.88954644345256, 174.96685793293122, 16.61099875397732, -96.93212717990885, 181.99035507847407, -110.80255352859031};

static float
ns_v2_polyval2d(float X, float Y, float C[16])
{
	float X2 = X * X;
	float X3 = X2 * X;
	float Y2 = Y * Y;
	float Y3 = Y2 * Y;
	return (
	    ((C[0]) + (C[1] * Y) + (C[2] * Y2) + (C[3] * Y3)) +
	    ((C[4] * X) + (C[5] * X * Y) + (C[6] * X * Y2) + (C[7] * X * Y3)) +
	    ((C[8] * X2) + (C[9] * X2 * Y) + (C[10] * X2 * Y2) +
	     (C[11] * X2 * Y3)) +
	    ((C[12] * X3) + (C[13] * X3 * Y) + (C[14] * X3 * Y2) +
	     (C[15] * X3 * Y3)));
}

vr::DistortionCoordinates_t northstar::driver::CHMD::ComputeDistortion(vr::EVREye eEye, float fU, float fV) {
    //todo: make the FoV correct...
    
    fU = 1.0f-fU;
    fV=1.0f-fV;
    float x_ray=0;
    float y_ray=0;
    if (eEye==0){
        // Blindly assuming that this is the left eye. Probably wrong.
        x_ray = ns_v2_polyval2d(fU, fV, left_uv_to_rect_x);
        y_ray = ns_v2_polyval2d(fU, fV, left_uv_to_rect_y);
    } else {
        x_ray = ns_v2_polyval2d(fU, fV, right_uv_to_rect_x);
        y_ray = ns_v2_polyval2d(fU, fV, right_uv_to_rect_y);
    }
    float left_ray_bound;
    float right_ray_bound;
    float up_ray_bound;
    float down_ray_bound;
    
    northstar::driver::CHMD::GetProjectionRaw(eEye, &left_ray_bound, &right_ray_bound, &up_ray_bound, &down_ray_bound); // Will result in violently weird FoV but should work for now.
	
    float u_eye = (x_ray + right_ray_bound) / (right_ray_bound - left_ray_bound);
	float v_eye = (y_ray + up_ray_bound) / (up_ray_bound - down_ray_bound);

    vr::DistortionCoordinates_t coordinates;
    coordinates.rfRed[0] = u_eye;
    coordinates.rfRed[1] = v_eye;
    coordinates.rfGreen[0] = u_eye;
    coordinates.rfGreen[1] = v_eye;
    coordinates.rfBlue[0] = u_eye;
    coordinates.rfBlue[1] = v_eye;
    return coordinates;
}

void northstar::driver::CHMD::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
    if(unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

// TODO: this should go in a thread
void northstar::driver::CHMD::RunFrame() {
    if (m_sOpenVRState.unObjectId != vr::k_unTrackedDeviceIndexInvalid)
        m_pVRServerDriverHost->TrackedDevicePoseUpdated(m_sOpenVRState.unObjectId, GetPose(), sizeof(vr::DriverPose_t));
}

const std::string_view& northstar::driver::CHMD::GetSerialNumber() const {
    return x_svSerialNumber;
}
