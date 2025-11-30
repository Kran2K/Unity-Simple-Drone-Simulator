using UnityEngine;

public class DroneCamera : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("추적할 드론(Cube)을 여기에 넣어")]
    public Transform target;

    [Header("Orbit Settings (마우스 회전)")]
    public float xSpeed = 120.0f;
    public float ySpeed = 120.0f;
    public float yMinLimit = -20f; // 바닥 뚫고 안 내려가게 제한
    public float yMaxLimit = 80f;  // 수직 위까지만

    [Header("Distance Settings (휠 줌)")]
    public float distance = 10.0f;
    public float minDistance = 2.0f;
    public float maxDistance = 50.0f;
    public float zoomSpeed = 5.0f;

    [Header("FOV Settings (키보드 + -)")]
    public float fovSpeed = 50.0f;
    public float minFov = 30.0f;
    public float maxFov = 100.0f;

    // 내부 각도 변수
    private float x = 0.0f;
    private float y = 0.0f;
    
    // 실제 적용할 거리 (부드러운 줌을 위해)
    private float currentDistance;
    private Camera _cam;

    void Start()
    {
        _cam = GetComponent<Camera>();

        // 시작할 때 현재 각도랑 거리 초기화
        Vector3 angles = transform.eulerAngles;
        x = angles.y;
        y = angles.x;

        currentDistance = distance;

        // 마우스 커서 숨기고 고정하고 싶으면 아래 주석 해제 (ESC 누르면 풀리게 따로 짜야 함)
        // Cursor.lockState = CursorLockMode.Locked;
    }

    void LateUpdate() // 중요: 타겟이 다 움직인 뒤에 카메라가 이동해야 부드러움
    {
        if (!target) return;

        // 1. 마우스 회전 (우클릭 상태에서만 돌리게 하려면 if(Input.GetMouseButton(1)) 추가)
        // 여기선 그냥 마우스 움직이면 바로 돌아가게 짬 (FPS 게임처럼)
        // 불편하면 if문으로 감싸라.
        if (Input.GetMouseButton(1)) // 우클릭 할 때만 시점 회전 (강추)
        {
            x += Input.GetAxis("Mouse X") * xSpeed * 0.02f;
            y -= Input.GetAxis("Mouse Y") * ySpeed * 0.02f;
            y = ClampAngle(y, yMinLimit, yMaxLimit);
        }

        // 2. 마우스 휠로 거리 조절 (물리적 거리)
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        distance -= scroll * zoomSpeed;
        distance = Mathf.Clamp(distance, minDistance, maxDistance);

        // 거리는 부드럽게 보간 (Lerp)
        currentDistance = Mathf.Lerp(currentDistance, distance, Time.deltaTime * 10f);

        // 3. 키보드 +/- 로 FOV 조절 (광각/망원)
        // Keypad Plus/Minus랑 알파벳 쪽 +(=), -(_) 둘 다 먹게 처리
        float fovChange = 0f;
        if (Input.GetKey(KeyCode.Plus) || Input.GetKey(KeyCode.KeypadPlus) || Input.GetKey(KeyCode.Equals))
            fovChange = -1f; // 줌인 (FOV 줄임)
        if (Input.GetKey(KeyCode.Minus) || Input.GetKey(KeyCode.KeypadMinus))
            fovChange = 1f;  // 줌아웃 (FOV 늘림)

        _cam.fieldOfView += fovChange * fovSpeed * Time.deltaTime;
        _cam.fieldOfView = Mathf.Clamp(_cam.fieldOfView, minFov, maxFov);

        // 4. 최종 위치 계산 및 적용
        Quaternion rotation = Quaternion.Euler(y, x, 0);
        Vector3 negDistance = new Vector3(0.0f, 0.0f, -currentDistance);
        
        // 타겟 위치에서 회전만큼 뒤로 빼고(negDistance), 높이 살짝 보정
        Vector3 position = rotation * negDistance + target.position + Vector3.up * 1.0f; // 중심점 살짝 위로

        transform.rotation = rotation;
        transform.position = position;
    }

    // 각도 360도 넘어가면 깔끔하게 정리해주는 유틸리티 함수
    public static float ClampAngle(float angle, float min, float max)
    {
        if (angle < -360F) angle += 360F;
        if (angle > 360F) angle -= 360F;
        return Mathf.Clamp(angle, min, max);
    }
}