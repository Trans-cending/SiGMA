import numpy as np

import numpy as np

def reconstruct_nearfield_H_upa_squeeze(
        a_b: np.ndarray,      # [num_paths]
        theta_r: np.ndarray,  # Rx Zenith [num_paths], local if tx/rx_forward is provided by caller
        phi_r: np.ndarray,    # Rx Azimuth [num_paths]
        theta_t: np.ndarray,  # Tx Zenith [num_paths]
        phi_t: np.ndarray,    # Tx Azimuth [num_paths]
        n_rx_rows: int, n_rx_cols: int,
        n_tx_rows: int, n_tx_cols: int,
        rx_rot_x: float = 0.0, rx_rot_y: float = 0.0, rx_rot_z: float = 0.0,
        tx_rot_x: float = 0.0, tx_rot_y: float = 0.0, tx_rot_z: float = 0.0,
        fc: float = 28e9,
        d_ratio: float = 0.5,
        if_filter: bool = True,
        rx_if_filter=None,
        tx_if_filter=None,
        tx_pos=None,
        rx_pos=None,
        tx_forward=None,
        rx_forward=None,
        first_interaction_pos=None,
        last_interaction_pos=None,
):
    """
    Near-field UPA channel reconstruction, drop-in compatible with
    reconstruct_farfield_H_upa_squeeze() plus optional geometry arguments.

    Purpose
    -------
    Far-field steering assumes all elements see the same path direction. For
    near-field reconstruction, each antenna element uses its true distance to
    the first Tx-side interaction point and the last Rx-side interaction point.

    Required geometry for true near-field
    -------------------------------------
    - tx_pos:                 Tx array phase-center position in GCS, shape (3,)
    - rx_pos:                 Rx array phase-center position in GCS, shape (3,)
    - first_interaction_pos:  per-path first point after Tx, shape (P,3)
                              For LOS this may be NaN/missing; rx_pos is used.
    - last_interaction_pos:   per-path last point before Rx, shape (P,3)
                              For LOS this may be NaN/missing; tx_pos is used.
    - tx_forward/rx_forward:  GCS unit vectors for array broadside +Z_lcs.

    Convention
    ----------
    Array LCS matches the farfield helper:
      +Z = forward / broadside, +X = right, +Y = down, UPA plane = XY.

    Filtering
    ---------
    if_filter=True removes paths whose first/last segment lies behind the
    corresponding array in LCS, i.e. local z < 0. This is the near-field analogue
    of the back-lobe filter in reconstruct_farfield_H_upa_squeeze().

    Fallback
    --------
    If tx_pos/rx_pos or interaction points are missing, the function still runs:
    missing per-path interaction points are replaced with the direct opposite
    endpoint (Tx uses rx_pos, Rx uses tx_pos), which preserves execution for LOS
    or for Sionna versions with unavailable vertices. For exact far-field
    equivalence, keep CHANNEL_RECONSTRUCTION_MODE=farfield in the generator.
    """
    import numpy as np

    a_b = np.asarray(a_b, dtype=np.complex128).reshape(-1)
    theta_r = np.asarray(theta_r, dtype=np.float64).reshape(-1)
    phi_r = np.asarray(phi_r, dtype=np.float64).reshape(-1)
    theta_t = np.asarray(theta_t, dtype=np.float64).reshape(-1)
    phi_t = np.asarray(phi_t, dtype=np.float64).reshape(-1)

    num_paths = int(a_b.shape[0])
    n_ue = int(n_rx_rows) * int(n_rx_cols)
    n_bs = int(n_tx_rows) * int(n_tx_cols)

    if num_paths == 0:
        return np.zeros((n_ue, n_bs), dtype=np.complex64)

    rx_filter = bool(if_filter if rx_if_filter is None else rx_if_filter)
    tx_filter = bool(if_filter if tx_if_filter is None else tx_if_filter)

    wavelength = 3e8 / float(fc)
    d = float(d_ratio) * wavelength
    k = 2.0 * np.pi / wavelength

    def _array_lcs_positions(n_rows, n_cols):
        col_idx = np.arange(int(n_cols), dtype=np.float64) - (int(n_cols) - 1) / 2.0
        row_idx = np.arange(int(n_rows), dtype=np.float64) - (int(n_rows) - 1) / 2.0
        x = np.tile(col_idx, int(n_rows)) * d
        y = np.repeat(row_idx, int(n_cols)) * d
        z = np.zeros_like(x)
        return np.stack([x, y, z], axis=0)  # (3, Nant)

    def _rot_basis_from_euler(rx, ry, rz):
        cz, sz = np.cos(rz), np.sin(rz)
        Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        cy, sy = np.cos(ry), np.sin(ry)
        Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
        cx, sx = np.cos(rx), np.sin(rx)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
        return Rz @ Ry @ Rx

    def _basis_from_forward(forward, fallback_rot):
        if forward is None:
            return fallback_rot
        f = np.asarray(forward, dtype=np.float64).reshape(3)
        nf = float(np.linalg.norm(f))
        if nf <= 1e-12:
            return fallback_rot
        f = f / nf
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        right = np.cross(f, world_up)
        if np.linalg.norm(right) < 1e-9:
            right = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        right = right / max(float(np.linalg.norm(right)), 1e-12)
        down = np.cross(f, right)
        down = down / max(float(np.linalg.norm(down)), 1e-12)
        return np.stack([right, down, f], axis=1)  # LCS columns in GCS

    def _angles_to_local_unit(theta, phi):
        return np.stack([
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(theta),
        ], axis=0)  # (3, P)

    def _valid_points(points):
        if points is None:
            return np.zeros((num_paths, 3), dtype=np.float64), np.zeros((num_paths,), dtype=bool)
        arr = np.asarray(points, dtype=np.float64)
        if arr.size == 0:
            return np.zeros((num_paths, 3), dtype=np.float64), np.zeros((num_paths,), dtype=bool)
        arr = arr.reshape(-1, 3)
        if arr.shape[0] < num_paths:
            pad = np.full((num_paths - arr.shape[0], 3), np.nan, dtype=np.float64)
            arr = np.vstack([arr, pad])
        arr = arr[:num_paths]
        valid = np.all(np.isfinite(arr), axis=1) & (np.linalg.norm(arr, axis=1) > 1e-9)
        return arr, valid

    r_lcs_rx = _array_lcs_positions(n_rx_rows, n_rx_cols)
    r_lcs_tx = _array_lcs_positions(n_tx_rows, n_tx_cols)

    R_fallback_rx = _rot_basis_from_euler(rx_rot_x, rx_rot_y, rx_rot_z)
    R_fallback_tx = _rot_basis_from_euler(tx_rot_x, tx_rot_y, tx_rot_z)
    B_rx = _basis_from_forward(rx_forward, R_fallback_rx)
    B_tx = _basis_from_forward(tx_forward, R_fallback_tx)

    # GCS element offsets from phase center. Columns are element offsets.
    r_gcs_rx = B_rx @ r_lcs_rx  # (3, n_ue)
    r_gcs_tx = B_tx @ r_lcs_tx  # (3, n_bs)

    # If positions are unavailable, fall back to far-field-equivalent steering
    # using the supplied local angles and zero rotations.
    if tx_pos is None or rx_pos is None:
        u_lcs_rx = _angles_to_local_unit(theta_r, phi_r)
        u_lcs_tx = _angles_to_local_unit(theta_t, phi_t)
        no_sig_rx = (u_lcs_rx[2, :] >= 0).astype(float) if rx_filter else np.ones(num_paths)
        no_sig_tx = (u_lcs_tx[2, :] >= 0).astype(float) if tx_filter else np.ones(num_paths)
        a_rx = np.exp(-1j * k * (r_lcs_rx.T @ u_lcs_rx)) / np.sqrt(n_ue) * no_sig_rx[np.newaxis, :]
        a_tx = np.exp(-1j * k * (r_lcs_tx.T @ u_lcs_tx)) / np.sqrt(n_bs) * no_sig_tx[np.newaxis, :]
        return ((a_rx * a_b[np.newaxis, :]) @ np.conj(a_tx).T * np.sqrt(n_ue * n_bs)).astype(np.complex64)

    tx_pos = np.asarray(tx_pos, dtype=np.float64).reshape(3)
    rx_pos = np.asarray(rx_pos, dtype=np.float64).reshape(3)
    first_pts, first_valid = _valid_points(first_interaction_pos)
    last_pts, last_valid = _valid_points(last_interaction_pos)

    # LOS / missing-vertex fallback per path.
    first_pts[~first_valid] = rx_pos[None, :]
    last_pts[~last_valid] = tx_pos[None, :]

    # Direction used only for back-lobe filtering.
    tx_dir_gcs = first_pts - tx_pos[None, :]
    rx_dir_gcs = last_pts - rx_pos[None, :]
    tx_dir_gcs = tx_dir_gcs / np.maximum(np.linalg.norm(tx_dir_gcs, axis=1, keepdims=True), 1e-12)
    rx_dir_gcs = rx_dir_gcs / np.maximum(np.linalg.norm(rx_dir_gcs, axis=1, keepdims=True), 1e-12)
    tx_dir_lcs = B_tx.T @ tx_dir_gcs.T
    rx_dir_lcs = B_rx.T @ rx_dir_gcs.T

    no_sig_tx = (tx_dir_lcs[2, :] >= 0).astype(float) if tx_filter else np.ones(num_paths)
    no_sig_rx = (rx_dir_lcs[2, :] >= 0).astype(float) if rx_filter else np.ones(num_paths)

    # Element locations in GCS: (Nant, 3)
    tx_elem = tx_pos[None, :] + r_gcs_tx.T
    rx_elem = rx_pos[None, :] + r_gcs_rx.T

    # Distance delta relative to phase center, per element/path.
    # Sign is chosen so the far-field limit matches reconstruct_farfield_H_upa_squeeze():
    # a_elem = exp(+j*k*(d_elem - d_center)).
    d_tx_center = np.linalg.norm(first_pts - tx_pos[None, :], axis=1)  # (P,)
    d_rx_center = np.linalg.norm(last_pts - rx_pos[None, :], axis=1)   # (P,)
    d_tx_elem = np.linalg.norm(tx_elem[:, None, :] - first_pts[None, :, :], axis=2)  # (n_bs, P)
    d_rx_elem = np.linalg.norm(rx_elem[:, None, :] - last_pts[None, :, :], axis=2)   # (n_ue, P)

    a_tx = np.exp(1j * k * (d_tx_elem - d_tx_center[None, :])) / np.sqrt(n_bs) * no_sig_tx[None, :]
    a_rx = np.exp(1j * k * (d_rx_elem - d_rx_center[None, :])) / np.sqrt(n_ue) * no_sig_rx[None, :]

    H = (a_rx * a_b[np.newaxis, :]) @ np.conj(a_tx).T
    return (H * np.sqrt(n_ue * n_bs)).astype(np.complex64)



def reconstruct_farfield_H_upa_squeeze(
        a_b: np.ndarray,      # [num_paths]
        theta_r: np.ndarray,  # Rx Zenith [num_paths]
        phi_r: np.ndarray,    # Rx Azimuth [num_paths]
        theta_t: np.ndarray,  # Tx Zenith [num_paths]
        phi_t: np.ndarray,    # Tx Azimuth [num_paths]
        n_rx_rows: int, n_rx_cols: int,
        n_tx_rows: int, n_tx_cols: int,
        rx_rot_x: float = 0.0, rx_rot_y: float = 0.0, rx_rot_z: float = 0.0,
        tx_rot_x: float = 0.0, tx_rot_y: float = 0.0, tx_rot_z: float = 0.0,
        fc: float = 28e9,
        d_ratio: float = 0.5,
        if_filter: bool = True,
        rx_if_filter=None,
        tx_if_filter=None,
):
    """
    【修正版】远场信道重构（向量化加速）
    严格对齐 farfield_upa_steering_vector 的定义：
    1. 阵列位于 LCS 的 XY 平面 (Z=0)。
    2. Forward 为 +Z，Up 为 -Y。
    """
    num_paths = len(a_b)
    n_ue = n_rx_rows * n_rx_cols
    n_bs = n_tx_rows * n_tx_cols
    
    rx_filter = bool(if_filter if rx_if_filter is None else rx_if_filter)
    tx_filter = bool(if_filter if tx_if_filter is None else tx_if_filter)

    wavelength = 3e8 / fc
    d = d_ratio * wavelength
    k = 2 * np.pi / wavelength
    
    # ==========================
    # 1. 构建 RX 阵列 (XY平面)
    # ==========================
    # 保持与循环版本完全一致的 Flatten 顺序 (Row-Major: 先遍历Col再遍历Row)
    col_idx_rx = np.arange(n_rx_cols) - (n_rx_cols - 1) / 2.0
    row_idx_rx = np.arange(n_rx_rows) - (n_rx_rows - 1) / 2.0
    
    # 修正：X对应Col, Y对应Row, Z=0
    x_pos_rx = np.tile(col_idx_rx, n_rx_rows) * d
    y_pos_rx = np.repeat(row_idx_rx, n_rx_cols) * d
    z_pos_rx = np.zeros_like(x_pos_rx)
    
    r_lcs_rx = np.stack([x_pos_rx, y_pos_rx, z_pos_rx])  # (3, n_ue)
    
    # RX 旋转矩阵 (R_z @ R_y @ R_x)
    cz, sz = np.cos(rx_rot_z), np.sin(rx_rot_z)
    R_Z_rx = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    
    cy, sy = np.cos(rx_rot_y), np.sin(rx_rot_y)
    R_Y_rx = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    
    cx, sx = np.cos(rx_rot_x), np.sin(rx_rot_x)
    R_X_rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    
    R_pose_rx = R_Z_rx @ R_Y_rx @ R_X_rx
    R_gcs2lcs_rx = R_pose_rx.T

    # ==========================
    # 2. 构建 TX 阵列 (XY平面)
    # ==========================
    col_idx_tx = np.arange(n_tx_cols) - (n_tx_cols - 1) / 2.0
    row_idx_tx = np.arange(n_tx_rows) - (n_tx_rows - 1) / 2.0
    
    x_pos_tx = np.tile(col_idx_tx, n_tx_rows) * d
    y_pos_tx = np.repeat(row_idx_tx, n_tx_cols) * d
    z_pos_tx = np.zeros_like(x_pos_tx)
    
    r_lcs_tx = np.stack([x_pos_tx, y_pos_tx, z_pos_tx])  # (3, n_bs)
    
    # TX 旋转矩阵
    cz, sz = np.cos(tx_rot_z), np.sin(tx_rot_z)
    R_Z_tx = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    
    cy, sy = np.cos(tx_rot_y), np.sin(tx_rot_y)
    R_Y_tx = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    
    cx, sx = np.cos(tx_rot_x), np.sin(tx_rot_x)
    R_X_tx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    
    R_pose_tx = R_Z_tx @ R_Y_tx @ R_X_tx
    R_gcs2lcs_tx = R_pose_tx.T

    # ==========================
    # 3. 计算 RX 导向矢量
    # ==========================
    # GCS 方向矢量 u (3, num_paths)
    u_gcs_rx = np.stack([
        np.sin(theta_r) * np.cos(phi_r), # x
        np.sin(theta_r) * np.sin(phi_r), # y
        np.cos(theta_r)                  # z
    ])
    
    # 转换到 LCS
    u_lcs_rx = R_gcs2lcs_rx @ u_gcs_rx
    
    # 修正背瓣过滤：LCS下前方是 +Z，因此检查 z 分量 (index 2) 是否 < 0
    # 注意：函数1是 if u[2] < 0 then 0.0。
    # 这里 if_filter=True 且 u[2] < 0 (背面) 则为0，否则为1。
    if rx_filter:
        no_sig_rx = (u_lcs_rx[2, :] >= 0).astype(float)
    else:
        no_sig_rx = np.ones(num_paths)
    
    # 计算相位 (LCS下平面波点积)
    # r_lcs_rx (3, n_ue), u_lcs_rx (3, num_paths) -> (n_ue, num_paths)
    phases_rx = k * (r_lcs_rx.T @ u_lcs_rx)
    a_ue_matrix = np.exp(-1j * phases_rx) / np.sqrt(n_ue) * no_sig_rx[np.newaxis, :]
    
    # ==========================
    # 4. 计算 TX 导向矢量
    # ==========================
    u_gcs_tx = np.stack([
        np.sin(theta_t) * np.cos(phi_t),
        np.sin(theta_t) * np.sin(phi_t),
        np.cos(theta_t)
    ])
    
    u_lcs_tx = R_gcs2lcs_tx @ u_gcs_tx
    
    if tx_filter:
        no_sig_tx = (u_lcs_tx[2, :] >= 0).astype(float)
    else:
        no_sig_tx = np.ones(num_paths)
        
    phases_tx = k * (r_lcs_tx.T @ u_lcs_tx)
    a_bs_matrix = np.exp(-1j * phases_tx) / np.sqrt(n_bs) * no_sig_tx[np.newaxis, :]
    
    # ==========================
    # 5. 信道重构
    # ==========================
    # H = sum( gain * a_rx * a_tx^H )
    # 利用广播机制：(N_rx, P) * (1, P) @ (P, N_tx) -> (N_rx, N_tx)
    H = (a_ue_matrix * a_b[np.newaxis, :]) @ np.conj(a_bs_matrix).T
    
    return H * np.sqrt(n_ue * n_bs)



def generate_gt_bartlett_front_mask(img_h: int = 90, img_w: int = 360) -> np.ndarray:
    """Return the Array-LCS front-hemisphere mask used by IF filtering.

    The Stage3 array local frame is +Z forward, +X right, +Y down.  Bartlett
    pixels use lon in [-pi, pi] and lat in [-pi/2, pi/2], so a pixel is in
    the physical front hemisphere exactly when u_z = cos(lat)*cos(lon) >= 0.
    Raw propagation paths remain intact; the mask aligns array/beam
    visualization with channel reconstruction.
    """
    u_vals = np.arange(int(img_w), dtype=np.float32)
    v_vals = np.arange(int(img_h), dtype=np.float32)
    grid_u, grid_v = np.meshgrid(u_vals, v_vals, indexing="xy")
    ndc_x = (2.0 * (grid_u + 0.5) / float(img_w)) - 1.0
    ndc_y = (2.0 * (grid_v + 0.5) / float(img_h)) - 1.0
    lon = ndc_x * np.pi
    lat = ndc_y * (np.pi * 0.5)
    u_z = np.cos(lat) * np.cos(lon)
    return (u_z >= -1e-7)


def generate_gt_bartlett_codebook(            
    num_rows: int, num_cols: int,             
    img_h: int = 90, img_w: int = 360,          
    d_ratio: float = 0.5, 
):
    """
    直接兼容 farfield_upa_steering_vector_3dgs 的 Bartlett 生成器。
    两者共享物理对齐的坐标系。
    """

    # 1. 像素 -> NDC -> Lat/Lon
    u_vals = np.arange(img_w, dtype=np.float32)
    v_vals = np.arange(img_h, dtype=np.float32)
    grid_u, grid_v = np.meshgrid(u_vals, v_vals, indexing='xy')

    ndc_x = (2.0 * (grid_u + 0.5) / img_w) - 1.0
    ndc_y = (2.0 * (grid_v + 0.5) / img_h) - 1.0

    lon = ndc_x * np.pi
    lat = ndc_y * (np.pi * 0.5)

    # 2. 像素对应的方向向量 u_lcs (Cam Space = Array LCS)
    # 这里的公式直接对应 3DGS 的辅助函数逻辑
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_lon = np.sin(lon)
    cos_lon = np.cos(lon)

    # 在 3DGS 坐标系下 (及新的 Array LCS):
    # Forward = +Z, Right = +X, Down = +Y
    # lat 是俯仰角 (Pixel Y 对应 lat)。
    # point3ToLonlatPixel 中: lat = asin(y / r)。这意味着 y 决定 lat。
    # ndc_y 从 -1 (Top) 到 1 (Bottom)。
    # Lat 从 -pi/2 (Top) 到 pi/2 (Bottom)。
    # y = sin(lat)。所以 Top y=-1, Bottom y=1。符合 +Y Down。
    
    u_y = np.sin(lat)              # +Y (Down)
    r_xz = np.cos(lat)
    u_x = r_xz * np.sin(lon)       # +X (Right)
    u_z = r_xz * np.cos(lon)       # +Z (Forward)

    wave_dir = np.stack([u_x.flatten(), u_y.flatten(), u_z.flatten()], axis=0) # (3, N_pixels)

    # 3. 阵元位置 (LCS: XY 平面)
    # 必须与 farfield 函数中的逻辑完全一致
    col_idx = np.arange(num_cols) - (num_cols - 1) / 2.0
    row_idx = np.arange(num_rows) - (num_rows - 1) / 2.0 # 正向，因为 Y-Down
    
    ant_x = np.tile(col_idx, num_rows)
    ant_y = np.repeat(row_idx, num_cols)
    # ant_z = 0
    
    ant_pos_xy = np.stack([ant_x, ant_y], axis=0) # (2, N_ant)

    # 4. 相位计算
    # phases = k * (x*u_x + y*u_y + z*u_z) -> z=0
    # wave_dir[0:2] 取 x, y 分量
    wave_dir_xy = wave_dir[0:2, :] 
    
    # 使用预计算的 codebook
    phases = 2 * np.pi * d_ratio * (ant_pos_xy.T @ wave_dir_xy)

    # 5. Bartlett 功率
    N_ant = num_rows * num_cols
    A = np.exp(-1j * phases) / np.sqrt(N_ant)
        

    return A

