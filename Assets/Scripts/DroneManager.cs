using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Collections.Concurrent;
using System;

// 1. 명령 패킷 (수신용)
[Serializable]
public class DronePacket
{
    public string type;  // "MODE", "CMD"
    public string mode;  // "POS", "VEL", "LAND", "TAKEOFF"
    public float x, y, z;
    public float yaw;
}

// 2. 텔레메트리 패킷 (송신용 - 상태 정보)
[Serializable]
public struct DroneTelemetry
{
    public double time;
    public string mode;       // 현재 모드
    public bool isArmed;      // 시동 여부 (추가됨)
    public float battery;     // 배터리 전압 (추가됨)
    public Vector3 position;  // Home 기준 로컬 위치
    public Vector3 velocity;
    public Vector3 acceleration;
    public Vector3 attitude;  // 자세 (Euler)
    public Vector3 angularVel;
}
public class DroneManager : MonoBehaviour
{
    [Header("Network Settings")]
    [SerializeField] private int listenPort = 8080;
    [SerializeField] private string targetIP = "127.0.0.1";
    [SerializeField] private int sendPort = 8081;
    [SerializeField] private float sendRate = 0.05f; // 20Hz

    [Header("Flight Parameters")]
    [SerializeField] private float takeoffHeight = 2.0f; // 이륙 높이
    [SerializeField] private float maxSpeed = 15f;
    [SerializeField] private float moveSmoothTime = 1.5f; // Position 모드 반응성
    [SerializeField] private float velAccFactor = 1.0f;   // Velocity 모드 가속성
    [SerializeField] private float rotSmoothTime = 1.0f;

    [Header("Visual & FX")]
    [SerializeField] private float maxTiltAngle = 25f;    // 이동 시 최대 기울기
    [SerializeField] private float tiltSensitivity = 2.0f;
    [SerializeField] private bool enableNoise = true;     // 호버링 노이즈 여부
    [SerializeField] private float noiseStrength = 0.1f;

    // --- State Variables ---
    public enum FlightMode { Disarmed, Position, Velocity, Takeoff, Land }
    public FlightMode CurrentMode { get; private set; } = FlightMode.Disarmed;
    public bool IsArmed { get; private set; } = false;

    public DroneTelemetry CurrentTelemetry { get; private set; }

    // 좌표계 기준점 (Home)
    private Vector3 _homePos;
    private float _homeYaw;

    // 목표값 (Target)
    private Vector3 _targetWorldPos; // 실제 유니티 월드 좌표 타겟
    private Vector3 _targetLocalVel;
    private float _targetYaw;

    // 물리 시뮬레이션용 (Damping Ref)
    private Vector3 _currentVel;
    private Vector3 _prevVel;
    private Vector3 _currentAccel;
    private float _yawVel;
    
    // 노이즈 시드
    private Vector3 _noiseSeed;
    
    // 배터리 (재미 요소)
    private float _batteryVoltage = 12.6f;

    // 네트워크
    private UdpClient _udpReceiver;
    private UdpClient _udpSender;
    private Thread _recvThread;
    private bool _isRunning = true;
    private ConcurrentQueue<DronePacket> _commandQueue = new ConcurrentQueue<DronePacket>();
    private float _lastSendTime;

    void Start()
    {
        // 1. Home Position 박제 (여기가 (0,0,0)이다)
        _homePos = transform.position;
        _homeYaw = transform.eulerAngles.y;

        // 초기 상태 설정
        _targetWorldPos = _homePos;
        _targetYaw = _homeYaw;
        _prevVel = Vector3.zero;
        
        // 노이즈 시드
        _noiseSeed = new Vector3(UnityEngine.Random.value * 100, UnityEngine.Random.value * 100, UnityEngine.Random.value * 100);

        // 네트워크 시작
        StartNetwork();
    }

    void OnDestroy()
    {
        _isRunning = false;
        _udpReceiver?.Close();
        _udpSender?.Close();
        if (_recvThread != null && _recvThread.IsAlive) _recvThread.Join(100);
    }

    private void StartNetwork()
    {
        _recvThread = new Thread(UdpReceiverWork);
        _recvThread.IsBackground = true;
        _recvThread.Start();
        _udpSender = new UdpClient();
        Debug.Log($"[Drone System] Initialized. Home at {_homePos}. Waiting for Commands...");
    }

    private void UdpReceiverWork()
    {
        try
        {
            _udpReceiver = new UdpClient(listenPort);
            IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, 0);
            while (_isRunning)
            {
                try
                {
                    byte[] data = _udpReceiver.Receive(ref remoteEP);
                    string json = Encoding.UTF8.GetString(data);
                    if (!string.IsNullOrEmpty(json))
                        _commandQueue.Enqueue(JsonUtility.FromJson<DronePacket>(json));
                }
                catch (SocketException) { if (!_isRunning) break; }
                catch { /* Ignore garbage */ }
            }
        }
        catch (Exception e) { Debug.LogError($"[UDP Bind Error] {e.Message}"); }
    }

    public void InjectCommand(DronePacket packet) => _commandQueue.Enqueue(packet);

    void Update()
    {
        ProcessCommands();
        UpdateDynamics();
        UpdateBattery();
        PublishTelemetry();
    }

    private void ProcessCommands()
    {
        while (_commandQueue.TryDequeue(out DronePacket p))
        {
            // 1. 모드 변경 명령 처리
            if (p.type == "MODE")
            {
                HandleModeChange(p.mode);
            }
            // 2. 제어 명령 처리
            else if (p.type == "CMD")
            {
                HandleControlCommand(p);
            }
        }
    }

    private void HandleModeChange(string modeStr)
    {
        switch (modeStr.ToUpper())
        {
            case "TAKEOFF":
                if (!IsArmed) ArmDrone();
                CurrentMode = FlightMode.Takeoff;
                // 이륙 목표: 홈 위치 + 지정 높이
                _targetWorldPos = _homePos + Vector3.up * takeoffHeight;
                // Yaw는 현재 유지
                _targetYaw = transform.eulerAngles.y; 
                Debug.Log("[Mode] TAKEOFF Sequence Initiated.");
                break;

            case "LAND":
                CurrentMode = FlightMode.Land;
                Debug.Log("[Mode] Landing...");
                break;

            case "POS":
                if (!IsArmed) { Debug.LogWarning("Cannot switch to POS: Drone Disarmed."); return; }
                CurrentMode = FlightMode.Position;
                // 모드 진입 시 튀는 것 방지: 현재 위치를 목표로
                _targetWorldPos = transform.position;
                break;

            case "VEL":
                if (!IsArmed) { Debug.LogWarning("Cannot switch to VEL: Drone Disarmed."); return; }
                CurrentMode = FlightMode.Velocity;
                _targetLocalVel = Vector3.zero;
                break;
        }
    }

    private void HandleControlCommand(DronePacket p)
    {
        if (!IsArmed) return; // 시동 안 걸리면 명령 무시

        if (CurrentMode == FlightMode.Position)
        {
            // 입력받은 x,y,z는 Home 기준 Local 좌표임. World로 변환 필요.
            // World Target = Home + Local Input
            Vector3 localCmd = new Vector3(p.x, p.y, p.z);
            _targetWorldPos = _homePos + localCmd;
            _targetYaw = p.yaw;
        }
        else if (CurrentMode == FlightMode.Velocity)
        {
            // 속도 명령은 World Frame 기준이라 가정 (혹은 Body Frame일 수도 있지만 여기선 NED/ENU World 기준)
            _targetLocalVel = new Vector3(p.x, p.y, p.z);
            _targetYaw = p.yaw; // 속도 모드에서도 Yaw는 각도 제어
        }
    }

    private void ArmDrone()
    {
        IsArmed = true;
        Debug.Log(">>> DRONE ARMED <<<");
    }

    private void DisarmDrone()
    {
        IsArmed = false;
        CurrentMode = FlightMode.Disarmed;
        _currentVel = Vector3.zero;
        _targetLocalVel = Vector3.zero;
        Debug.Log(">>> DRONE DISARMED <<<");
    }

    private void UpdateDynamics()
    {
        float dt = Time.deltaTime;
        Vector3 nextPos = transform.position;

        // Disarmed 상태 처리
        if (!IsArmed)
        {
            if(transform.position.y > _homePos.y) 
                transform.position += Vector3.down * 5f * dt;
            return;
        }

        // --- 1. 위치/속도 계산 (기존과 동일) ---
        switch (CurrentMode)
        {
            case FlightMode.Takeoff:
            case FlightMode.Position:
                nextPos = Vector3.SmoothDamp(transform.position, _targetWorldPos, ref _currentVel, moveSmoothTime, maxSpeed);
                if (CurrentMode == FlightMode.Takeoff && Vector3.Distance(transform.position, _targetWorldPos) < 0.1f)
                    CurrentMode = FlightMode.Position;
                break;

            case FlightMode.Velocity:
                _currentVel = Vector3.Lerp(_currentVel, _targetLocalVel, dt * velAccFactor);
                nextPos += _currentVel * dt;
                break;

            case FlightMode.Land:
                if (transform.position.y > _homePos.y + 0.05f)
                {
                    nextPos += Vector3.down * 1.0f * dt;
                    _currentVel = Vector3.down * 1.0f;
                }
                else
                {
                    DisarmDrone();
                    return;
                }
                break;
        }

        // --- 2. 노이즈 (기존과 동일) ---
        if (enableNoise && IsArmed && CurrentMode != FlightMode.Land)
        {
            float t = Time.time;
            float nx = (Mathf.PerlinNoise(_noiseSeed.x, t) - 0.5f) * noiseStrength;
            float ny = (Mathf.PerlinNoise(_noiseSeed.y, t) - 0.5f) * noiseStrength;
            float nz = (Mathf.PerlinNoise(_noiseSeed.z, t) - 0.5f) * noiseStrength;
            nextPos += new Vector3(nx, ny, nz) * dt;
        }

        // --- 3. 자세 제어 (여기가 핵심 수정) ---
        
        // 가속도 계산
        _currentAccel = (_currentVel - _prevVel) / dt;
        _prevVel = _currentVel;

        // [수정 1] Yaw 회전: SmoothDamp 결과를 Slerp 없이 바로 쓴다.
        // rotSmoothTime을 0.1이나 0.05로 주면 즉각 반응함.
        float currentYaw = Mathf.SmoothDampAngle(transform.eulerAngles.y, _targetYaw, ref _yawVel, rotSmoothTime);

        // [수정 2] Tilt (기울임): 가속도 기반 목표 기울기 계산
        Vector3 localAccel = transform.InverseTransformDirection(_currentAccel);
        float targetPitch = Mathf.Clamp(localAccel.z * tiltSensitivity, -maxTiltAngle, maxTiltAngle);
        float targetRoll = Mathf.Clamp(-localAccel.x * tiltSensitivity, -maxTiltAngle, maxTiltAngle);

        // [수정 3] Tilt 적용 방식 변경
        // Yaw는 이미 부드러우니 그대로 쓰고, Tilt만 현재 상태에서 목표 상태로 부드럽게 보간한다.
        // 드론의 현재 Pitch/Roll을 가져와서 목표값으로 Lerp 해준다.
        
        // 현재 드론의 로컬 오일러 각도 (짐벌 락 방지를 위해 헬퍼 함수 쓰거나 직접 접근)
        Vector3 currentEuler = transform.eulerAngles;
        
        // 각도 보정 (0~360 -> -180~180)
        float currentPitch = (currentEuler.x > 180) ? currentEuler.x - 360 : currentEuler.x;
        float currentRoll = (currentEuler.z > 180) ? currentEuler.z - 360 : currentEuler.z;

        // Tilt만 부드럽게 (반응성 5.0f -> 10.0f 정도로 조절 가능)
        float newPitch = Mathf.Lerp(currentPitch, targetPitch, dt * 5.0f);
        float newRoll = Mathf.Lerp(currentRoll, targetRoll, dt * 5.0f);

        // [최종 적용] Slerp 없이 직관적으로 합성
        transform.rotation = Quaternion.Euler(newPitch, currentYaw, newRoll);
        
        // 위치 적용
        transform.position = nextPos;
    }

    private void UpdateBattery()
    {
        if (IsArmed) _batteryVoltage -= Time.deltaTime * 0.01f; // 비행 중 소모
        else _batteryVoltage += Time.deltaTime * 0.005f; // 대기 중 (충전이라 치자)
        _batteryVoltage = Mathf.Clamp(_batteryVoltage, 10.0f, 12.6f);
    }

    private void PublishTelemetry()
    {
        if (Time.time - _lastSendTime < sendRate) return;
        _lastSendTime = Time.time;

        // 중요: 텔레메트리는 Local 좌표(Home 기준)로 변환해서 보냄
        Vector3 localPos = transform.position - _homePos;

        CurrentTelemetry = new DroneTelemetry
        {
            time = Time.timeSinceLevelLoad,
            mode = CurrentMode.ToString(),
            isArmed = IsArmed,
            battery = (float)Math.Round(_batteryVoltage, 2),
            position = localPos,       // Local Position
            velocity = _currentVel,
            acceleration = _currentAccel,
            attitude = transform.eulerAngles,
            angularVel = new Vector3(0, _yawVel, 0)
        };

        try
        {
            string json = JsonUtility.ToJson(CurrentTelemetry);
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            _udpSender.Send(bytes, bytes.Length, targetIP, sendPort);
        }
        catch { /* UDP Send Error skip */ }
    }
}