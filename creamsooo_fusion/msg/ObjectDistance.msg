Header header
uint32 id             # 객체 ID
string class_name     # 객체 클래스 이름
float32 distance      # 객체까지의 거리 (미터)
geometry_msgs/Point center  # 객체 중심점 3D 좌표
bool emergency        # 긴급 정지 필요 여부 (1m 이내인 경우 true) 