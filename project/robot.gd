extends CharacterBody3D

@export var linear_speed := 2.6
@export var angular_speed := 4.0
@export var linear_acc := 8.6
@export var angular_acc := 30.0

var angular_velocity := 0.0
var interface := ControlInterface.new()

func _ready() -> void:
    interface.base_link_frame = "base_footprint"
    interface.covariance_x = 0.001
    interface.covariance_y = 0.001
    interface.covariance_angular = 0.001
    interface.cmd_vel_received.connect(_on_cmd_received)

func _on_cmd_received() -> void:
    pass


func _physics_process(delta: float) -> void:
    var linear_cmd_local := Input.get_vector("move_backward", "move_forward", "move_right", "move_left") * linear_speed + interface.get_linear_cmd()
    var linear_cmd_global := global_transform.basis * Vector3(linear_cmd_local.x, linear_cmd_local.y, 0)
    var angular_cmd_global := Input.get_axis("turn_right", "turn_left") * angular_speed + interface.get_angular_cmd()
    
    if (not linear_cmd_global.is_finite()) or is_nan(angular_cmd_global): return
    
    var linear_diff := linear_cmd_global - velocity
    if linear_diff.length() < linear_acc * delta:
        velocity = linear_cmd_global
    else:
        velocity += linear_diff.normalized() * linear_acc * delta
    move_and_slide()

    var angular_diff := angular_cmd_global - angular_velocity
    if abs(angular_diff) < angular_acc * delta:
        angular_velocity = angular_cmd_global
    else:
        angular_velocity += sign(angular_diff) * angular_acc * delta
    global_rotate(Vector3(0, 1, 0), angular_velocity * delta)

    var local_vel := velocity * global_transform.basis
    interface.publish_odom(Vector2(local_vel.x, local_vel.y), angular_velocity)
