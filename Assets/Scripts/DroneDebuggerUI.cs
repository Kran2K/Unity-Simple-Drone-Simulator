using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class DroneDebuggerUI : MonoBehaviour
{
    [Header("Ref")]
    [SerializeField] private DroneManager manager;

    [Header("Controls")]
    [SerializeField] private TMP_Dropdown modeDropdown;
    [SerializeField] private TMP_InputField inputX, inputY, inputZ, inputYaw;
    [SerializeField] private Button sendBtn;

    [Header("UI Groups")]
    [SerializeField] private GameObject hudPanel;      // 게임 중 표시되는 메인 HUD (텍스트, 버튼 등)
    [SerializeField] private GameObject settingsPanel; // ESC 메뉴 (배경에 반투명 패널 포함 권장)

    [Header("Settings UI")]
    [SerializeField] private TMP_InputField ipInput;
    [SerializeField] private TMP_InputField listenPortInput;
    [SerializeField] private TMP_InputField sendPortInput;
    [SerializeField] private TMP_InputField fpsInput;
    [SerializeField] private Button applyNetworkBtn;
    [SerializeField] private Button applyFpsBtn;
    [SerializeField] private Button restartBtn;
    [SerializeField] private Button quitBtn;

    [Header("Display")]
    [SerializeField] private TextMeshProUGUI statusText;

    private bool isSettingsOpen = false;

    private void Start()
    {
        // 드롭다운 옵션 초기화 (POS, VEL, TAKEOFF, LAND)
        modeDropdown.ClearOptions();
        modeDropdown.AddOptions(new System.Collections.Generic.List<string> { "POS", "VEL", "TAKEOFF", "LAND" });

        sendBtn.onClick.AddListener(OnSend);
        modeDropdown.onValueChanged.AddListener(OnModeChange);
        
        // 입력 필드 초기값 설정
        inputX.text = "0"; inputY.text = "5"; inputZ.text = "0"; inputYaw.text = "0";

        // Settings UI Init
        if (settingsPanel != null)
        {
            settingsPanel.SetActive(false);
            
            // Init values
            ipInput.text = manager.TargetIP;
            listenPortInput.text = manager.ListenPort.ToString();
            sendPortInput.text = manager.SendPort.ToString();
            fpsInput.text = Application.targetFrameRate.ToString();

            // Add Listeners
            applyNetworkBtn.onClick.AddListener(OnApplyNetwork);
            applyFpsBtn.onClick.AddListener(OnApplyFps);
            restartBtn.onClick.AddListener(OnRestartSim);
            quitBtn.onClick.AddListener(OnQuitSim);
        }

        // 초기 상태: HUD 켜기, 설정 끄기
        if (hudPanel != null) hudPanel.SetActive(true);
    }

    private void Update()
    {
        // Toggle Settings with ESC
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            ToggleSettings();
        }

        if (manager == null) return;

        // Settings UI State Update
        if (isSettingsOpen && settingsPanel != null)
        {
            bool isConnected = manager.IsConnected;
            var btnText = applyNetworkBtn.GetComponentInChildren<TextMeshProUGUI>();
            if (btnText != null) btnText.text = isConnected ? "Close Socket" : "Open Socket";

            // 연결 중에는 입력 필드 비활성화
            ipInput.interactable = !isConnected;
            listenPortInput.interactable = !isConnected;
            sendPortInput.interactable = !isConnected;
        }

        // HUD가 켜져 있을 때만 상태 업데이트
        if (hudPanel != null && hudPanel.activeSelf)
        {
            var t = manager.CurrentTelemetry;

            // UI에 표시할 드론 상태 텍스트
            string status = $"<size=120%><b>DRONE STATUS</b></size>\n" +
                            $"----------------------\n" +
                            $"<b>MODE :</b> <color={(t.isArmed ? "green" : "red")}>{t.mode}</color>\n" +
                            $"<b>ARMED:</b> {(t.isArmed ? "YES" : "NO")}\n" +
                            $"<b>BATT :</b> {t.battery} V\n" +
                            $"<b>DIST_BOTTOM :</b> {(t.dist_bottom < 0 ? "N/A" : t.dist_bottom.ToString("F2") + " m")}\n\n" +
                            $"<b>[LOCAL POS] (Home Ref)</b>\n" +
                            $"X: {t.position.x:F2}  Y: {t.position.y:F2}  Z: {t.position.z:F2}\n\n" +
                            $"<b>[VELOCITY]</b>\n" +
                            $"X: {t.velocity.x:F2}  Y: {t.velocity.y:F2}  Z: {t.velocity.z:F2}\n\n" +
                            $"<b>[ACCELERATION]</b>\n" +
                            $"X: {t.acceleration.x:F2}  Y: {t.acceleration.y:F2}  Z: {t.acceleration.z:F2}\n\n" +
                            $"<b>[ATTITUDE]</b>\n" +
                            $"Roll: {t.attitude.x:F1}  Pitch: {t.attitude.y:F1}  Yaw: {t.attitude.z:F1}\n\n" +
                            $"<b>[ANGULAR VEL]</b>\n" +
                            $"X: {t.angularVel.x:F1}  Y: {t.angularVel.y:F1}  Z: {t.angularVel.z:F1}";
            
            statusText.text = status;
        }
    }

    private void ToggleSettings()
    {
        if (settingsPanel == null) return;
        
        isSettingsOpen = !isSettingsOpen;
        
        // 설정 패널 토글
        settingsPanel.SetActive(isSettingsOpen);

        // 메인 HUD 토글 (설정이 켜지면 꺼짐)
        if (hudPanel != null)
        {
            hudPanel.SetActive(!isSettingsOpen);
        }

        if (isSettingsOpen)
        {
            Time.timeScale = 0f; // Pause
        }
        else
        {
            Time.timeScale = 1f; // Resume
        }
    }

    private void OnApplyNetwork()
    {
        if (manager.IsConnected)
        {
            // 이미 연결된 상태면 닫기 (토글)
            manager.StopNetwork();
            Debug.Log("[UI] Network Socket Closed.");
        }
        else
        {
            // 닫힌 상태면 입력값 적용해서 열기
            string ip = ipInput.text;
            int.TryParse(listenPortInput.text, out int listen);
            int.TryParse(sendPortInput.text, out int send);

            manager.UpdateNetworkSettings(ip, listen, send);
            Debug.Log($"[UI] Network Started: {ip}, Listen: {listen}, Send: {send}");
        }
    }

    private void OnApplyFps()
    {
        int.TryParse(fpsInput.text, out int fps);
        if (fps > 0)
        {
            Application.targetFrameRate = fps;
            Debug.Log($"[UI] FPS Limit set to {fps}");
        }
    }

    private void OnRestartSim()
    {
        Debug.Log("[UI] Restarting Simulation...");
        manager.StopNetwork(); // 안전하게 네트워크 종료
        Time.timeScale = 1f;   // 시간 원래대로
        
        // 현재 씬 재로딩
        UnityEngine.SceneManagement.SceneManager.LoadScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene().buildIndex);
    }

    private void OnQuitSim()
    {
        Debug.Log("[UI] Quitting Application...");
        manager.StopNetwork();
        
        #if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
        #else
            Application.Quit();
        #endif
    }

    private void OnSend()
    {
        float.TryParse(inputX.text, out float x);
        float.TryParse(inputY.text, out float y);
        float.TryParse(inputZ.text, out float z);
        float.TryParse(inputYaw.text, out float yaw);

        DroneCommand p = new DroneCommand { type = "CMD", x = x, y = y, z = z, yaw = yaw };
        manager.InjectCommand(p);
    }

    private void OnModeChange(int idx)
    {
        string m = modeDropdown.options[idx].text;
        manager.InjectCommand(new DroneCommand { type = "MODE", mode = m });
    }
}