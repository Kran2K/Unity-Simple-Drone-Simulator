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

    [Header("Display")]
    [SerializeField] private TextMeshProUGUI statusText;

    private void Start()
    {
        // 드롭다운 옵션 초기화 (POS, VEL, TAKEOFF, LAND)
        modeDropdown.ClearOptions();
        modeDropdown.AddOptions(new System.Collections.Generic.List<string> { "POS", "VEL", "TAKEOFF", "LAND" });

        sendBtn.onClick.AddListener(OnSend);
        modeDropdown.onValueChanged.AddListener(OnModeChange);
        
        // 입력 필드 초기값 설정
        inputX.text = "0"; inputY.text = "5"; inputZ.text = "0"; inputYaw.text = "0";
    }

    private void Update()
    {
        if (manager == null) return;
        var t = manager.CurrentTelemetry;

        // UI에 표시할 드론 상태 텍스트
        string status = $"<size=110%><b>DRONE STATUS</b></size>\n" +
                        $"----------------------\n" +
                        $"<b>MODE :</b> <color={(t.isArmed ? "green" : "red")}>{t.mode}</color>\n" +
                        $"<b>ARMED:</b> {(t.isArmed ? "YES" : "NO")}\n" +
                        $"<b>BATT :</b> {t.battery} V\n" +
                        $"<b>GND_DIST :</b> {(t.dist_bottom < 0 ? "N/A" : t.dist_bottom.ToString("F2") + " m")}\n\n" +
                        $"<b>[LOCAL POS] (Home Ref)</b>\n" +
                        $"X: {t.position.x:F2}  Y: {t.position.y:F2}  Z: {t.position.z:F2}\n\n" +
                        $"<b>[VELOCITY]</b>\n" +
                        $"X: {t.velocity.x:F2}  Y: {t.velocity.y:F2}  Z: {t.velocity.z:F2}\n\n" +
                        $"<b>[ATTITUDE]</b>\n" +
                        $"R: {t.attitude.x:F1}  P: {t.attitude.z:F1}  Y: {t.attitude.y:F1}"; // Roll/Pitch 순서에 유의 (Unity는 X가 Pitch일 수 있음)
        
        statusText.text = status;
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