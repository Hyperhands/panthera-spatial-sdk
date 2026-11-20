A2L1Description — 从 DVT2 a2 提取的左右手模型

入口：urdf/left_hand.urdf、urdf/right_hand.urdf。
资源：mesh/visual/casbot_hand/ 与 mesh/collision/casbot_hand/；材质与 OBJ 同目录。
整体移动本文件夹即可；URDF 使用 ../mesh/ 相对路径，不依赖原仓库。

每手以 left/right_palm_base_link 为根：17 links，11 转动关节，5 固定关节，
其中 5 个转动关节为 mimic，从而有 6 个独立驱动关节。源质量每手 0.61699671 kg。
保留 a2 的 link/joint 名称、惯量、原点、轴、限位、阻尼、摩擦和 mimic；
未混入新收到的 P1L URDF 参数，未根据左右镜像猜测或修改参数。

安装到机器人时，源模型中 flange_frame → hand_mount_frame 为单位变换，
hand_mount_frame → palm_base_link 为 xyz=(0,0,0.0095)m、rpy=(0,0,0)。
本包不包含腕部或法兰实体；源安装关节完整 XML 见 source_and_checks.json。
使用已有同名法兰接入即可，不能重复加入手部负载。

注意：a2 中部分从动关节的倍率使主关节全范围与从动限位不兼容。
本次忠实保留源限位，未擅自扩大或缩小；双方限位共同允许的主关节范围
已计算在 source_and_checks.json 的 mimic_compatible_driver_limits_rad 中。
使用者需使加载器启用 mimic；把 11 个关节当作独立自由度会改变模型。
a2 的 mimic 参数不同于新收到的 P1L 文件，本包不是其控制标定修正版。

这些是源 CAD 模型参数，未做实物称重、装配或控制参数验证。
未接入机器人、未更改 casbot_calib 默认模型。本次验收为非视觉模型/资源检查。
