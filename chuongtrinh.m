function robot_premium_gui()
    % =========================================================================
    % MATLAB GUI: TÍCH HỢP ĐỘNG HỌC, QUY HOẠCH QUỸ ĐẠO & GIAO DIỆN CÔNG NGHIỆP
    % (PHIÊN BẢN PREMIUM)
    % =========================================================================
    
    %% 1. THÔNG SỐ CƠ KHÍ & GIỚI HẠN
    L1 = 0.40; % Chiều cao trụ (m)
    L3 = 0.25; % Chiều dài tay gắp (m)
    
    q1_lim = [-90, 90];     % Độ
    q2_lim = [0.10, 0.30];  % Mét
    q3_lim = [-75, 75];     % Độ
    
    % Biến toàn cục lưu vết quỹ đạo
    trace_x = [];
    trace_y = [];
    trace_z = [];
    
    %% 2. TẠO CỬA SỔ GIAO DIỆN CÔNG NGHIỆP (Nới rộng cửa sổ)
    fig = uifigure('Name', 'Industrial Robot Control Center (Pro Version)', 'Position', [50, 50, 1350, 750]);
    fig.Color = [0.94 0.94 0.96]; % Màu nền xám sang trọng
    
    % Khung đồ thị 3D
    ax = uiaxes(fig, 'Position', [400, 20, 700, 700]);
    hold(ax, 'on'); grid(ax, 'minor'); axis(ax, 'equal');
    view(ax, 35, 35);
    xlim(ax, [-0.6 0.6]); ylim(ax, [-0.6 0.6]); zlim(ax, [0 0.5]);
    title(ax, 'MÔ PHỎNG 3D VÀ ĐIỀU KHIỂN QUỸ ĐẠO ROBOT', 'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.5]);
    xlabel(ax, 'Trục X (m)', 'FontWeight', 'bold'); 
    ylabel(ax, 'Trục Y (m)', 'FontWeight', 'bold'); 
    zlabel(ax, 'Trục Z (m)', 'FontWeight', 'bold');
    
    %% 3. VẼ KHÔNG GIAN LÀM VIỆC (WORKSPACE POINT CLOUD)
    [Q1, Q2, Q3] = ndgrid(linspace(q1_lim(1)*pi/180, q1_lim(2)*pi/180, 25), ...
                          linspace(q2_lim(1), q2_lim(2), 8), ...
                          linspace(q3_lim(1)*pi/180, q3_lim(2)*pi/180, 25));
    X_ws = -Q2(:).*sin(Q1(:)) + L3.*cos(Q1(:)+Q3(:));
    Y_ws = Q2(:).*cos(Q1(:)) + L3.*sin(Q1(:)+Q3(:));
    
    C_ws = sqrt(X_ws.^2 + Y_ws.^2); 
    scatter3(ax, X_ws, Y_ws, L1*ones(size(X_ws)), 6, C_ws, 'filled', 'MarkerFaceAlpha', 0.35, 'MarkerEdgeAlpha', 0.35);
    colormap(ax, 'parula');
    
    %% 4. KHỞI TẠO ĐỐI TƯỢNG ROBOT VÀ VẾT QUỸ ĐẠO
    h_base = plot3(ax, [0 0], [0 0], [0 L1], 'k-', 'LineWidth', 14); 
    h_link2 = plot3(ax, [0 0], [0 0], [L1 L1], '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 10); 
    h_link3 = plot3(ax, [0 0], [0 0], [L1 L1], '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 8); 
    h_joints = scatter3(ax, [], [], [], 150, 'k', 'filled', 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    h_tcp = plot3(ax, 0, 0, 0, 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5); 
    
    h_trace = plot3(ax, NaN, NaN, NaN, 'm--', 'LineWidth', 2.5); % Đường vẽ vết quỹ đạo
    
    %% 5. BẢNG ĐIỀU KHIỂN ĐỘNG HỌC THUẬN (MANUAL)
    pnl_fk = uipanel(fig, 'Title', 'Động học Thuận (Manual Control)', 'Position', [20, 420, 350, 300], 'FontSize', 12, 'FontWeight', 'bold');
    uilabel(pnl_fk, 'Text', 'q1: Khớp quay đế (Độ)', 'Position', [15, 230, 180, 22]);
    sld_q1 = uislider(pnl_fk, 'Position', [20, 210, 300, 3], 'Limits', q1_lim, 'Value', 0);
    
    uilabel(pnl_fk, 'Text', 'q2: Khớp trượt ngang (m)', 'Position', [15, 150, 180, 22]);
    sld_q2 = uislider(pnl_fk, 'Position', [20, 130, 300, 3], 'Limits', q2_lim, 'Value', 0.2);
    
    uilabel(pnl_fk, 'Text', 'q3: Khớp quay cổ tay (Độ)', 'Position', [15, 70, 180, 22]);
    sld_q3 = uislider(pnl_fk, 'Position', [20, 50, 300, 3], 'Limits', q3_lim, 'Value', 0);
    
    sld_q1.ValueChangedFcn = @(sld,event) run_FK();
    sld_q2.ValueChangedFcn = @(sld,event) run_FK();
    sld_q3.ValueChangedFcn = @(sld,event) run_FK();
    
    %% 6. BẢNG ĐIỀU KHIỂN ĐỘNG HỌC NGƯỢC (AUTO TRAJECTORY)
    pnl_ik = uipanel(fig, 'Title', 'Nhiệm vụ (Trajectory Planning)', 'Position', [20, 120, 350, 280], 'FontSize', 12, 'FontWeight', 'bold');
    uilabel(pnl_ik, 'Text', 'X đích (m):', 'Position', [20, 210, 100, 25], 'FontWeight', 'bold');
    edt_x = uieditfield(pnl_ik, 'numeric', 'Position', [120, 210, 80, 25], 'Value', 0.25);
    
    uilabel(pnl_ik, 'Text', 'Y đích (m):', 'Position', [20, 170, 100, 25], 'FontWeight', 'bold');
    edt_y = uieditfield(pnl_ik, 'numeric', 'Position', [120, 170, 80, 25], 'Value', 0.2);
    
    uilabel(pnl_ik, 'Text', 'Góc Phi (Độ):', 'Position', [20, 130, 100, 25], 'FontWeight', 'bold');
    edt_phi = uieditfield(pnl_ik, 'numeric', 'Position', [120, 130, 80, 25], 'Value', 0.0);
    
    % Phần hiển thị giá trị IK tính toán chính xác
    uilabel(pnl_ik, 'Text', 'Kết quả IK:', 'Position', [220, 240, 100, 25], 'FontWeight', 'bold', 'FontColor', [0 0.4470 0.7410]);
    lbl_ik_q1 = uilabel(pnl_ik, 'Text', 'q1: ---', 'Position', [220, 210, 120, 25]);
    lbl_ik_q2 = uilabel(pnl_ik, 'Text', 'q2: ---', 'Position', [220, 170, 120, 25]);
    lbl_ik_q3 = uilabel(pnl_ik, 'Text', 'q3: ---', 'Position', [220, 130, 120, 25]);
    
    btn_ik = uibutton(pnl_ik, 'push', 'Text', 'CHẠY AUTO (ANIMATION)', 'Position', [20, 70, 300, 40], 'BackgroundColor', '#A2142F', 'FontColor', 'w', 'FontWeight', 'bold');
    btn_clear = uibutton(pnl_ik, 'push', 'Text', 'XÓA VẾT', 'Position', [20, 20, 140, 35]);
    btn_home = uibutton(pnl_ik, 'push', 'Text', 'VỀ HOME', 'Position', [180, 20, 140, 35]);
    
    btn_ik.ButtonPushedFcn = @(btn,event) run_IK_animation();
    btn_clear.ButtonPushedFcn = @(btn,event) clear_trace();
    btn_home.ButtonPushedFcn = @(btn,event) go_home();
    
    %% 7. BẢNG ĐỒNG HỒ CÔNG NGHIỆP (SMART GAUGES)
    pnl_gauge = uipanel(fig, 'Title', 'Cảm biến khớp (Real-time)', 'Position', [1120, 20, 210, 700], 'FontSize', 12, 'FontWeight', 'bold');
    
    gauge_q1 = uigauge(pnl_gauge, 'circular', 'Position', [30, 520, 150, 150], 'Limits', q1_lim);
    uilabel(pnl_gauge, 'Text', 'Khớp 1 (Độ)', 'Position', [70, 500, 100, 20], 'FontWeight', 'bold');
    lbl_val_q1 = uilabel(pnl_gauge, 'Text', '0.00°', 'Position', [55, 480, 100, 20], 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontColor', 'b');
    
    gauge_q2 = uigauge(pnl_gauge, 'linear', 'Position', [80, 300, 50, 150], 'Limits', q2_lim);
    uilabel(pnl_gauge, 'Text', 'Khớp 2 (m)', 'Position', [75, 270, 100, 20], 'FontWeight', 'bold');
    lbl_val_q2 = uilabel(pnl_gauge, 'Text', '0.00 m', 'Position', [55, 250, 100, 20], 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontColor', 'b');
    
    gauge_q3 = uigauge(pnl_gauge, 'circular', 'Position', [30, 80, 150, 150], 'Limits', q3_lim);
    uilabel(pnl_gauge, 'Text', 'Khớp 3 (Độ)', 'Position', [70, 60, 100, 20], 'FontWeight', 'bold');
    lbl_val_q3 = uilabel(pnl_gauge, 'Text', '0.00°', 'Position', [55, 40, 100, 20], 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontColor', 'b');
    
    %% 8. THANH TRẠNG THÁI & KIỂM TRA ĐIỂM KỲ DỊ (JACOBIAN)
    lbl_status = uilabel(fig, 'Text', 'Hệ thống sẵn sàng.', 'Position', [20, 60, 350, 40], 'FontSize', 14, 'FontWeight', 'bold', 'FontColor', '#77AC30');
    lbl_jacob = uilabel(fig, 'Text', 'Det(J): ---', 'Position', [20, 20, 350, 40], 'FontSize', 12, 'FontWeight', 'bold', 'FontColor', 'b');
    
    %% ================= HÀM XỬ LÝ (CALLBACKS) ==================
    
    function update_gauges_and_jacob(q1, q2, q3)
        % Cập nhật đồng hồ đo
        gauge_q1.Value = q1 * 180/pi;
        gauge_q2.Value = q2;
        gauge_q3.Value = q3 * 180/pi;
        
        % Cập nhật giá trị số chính xác
        lbl_val_q1.Text = sprintf('%.2f°', q1 * 180/pi);
        lbl_val_q2.Text = sprintf('%.4f m', q2);
        lbl_val_q3.Text = sprintf('%.2f°', q3 * 180/pi);
        
        % Đẳng cấp: Tính định thức ma trận Jacobian 3x3 để xét điểm kỳ dị
        % Ma trận Jacobian tính đạo hàm riêng của X, Y, Phi theo q1, q2, q3
        % Sau khi rút gọn bằng giải tích, det(J) của cơ cấu R-P-R này là -q2
        detJ = -q2; 
        
        lbl_jacob.Text = sprintf('Kiểm tra Kỳ dị Det(J): %.4f', detJ);
        
        % Chú ý: Vì giới hạn q2 thuộc [0.1, 0.3], det(J) luôn khác 0!
        % Điều này chứng tỏ robot R-P-R này hoàn toàn không có điểm kỳ dị 
        % (Singularity-free) bên trong không gian công tác của nó.
        lbl_jacob.FontColor = 'b';
    end
    
    function run_FK()
        q1_deg = sld_q1.Value;
        q2 = sld_q2.Value;
        q3_deg = sld_q3.Value;
        
        q1 = q1_deg * pi/180;
        q3 = q3_deg * pi/180;
        
        x = -q2 * sin(q1) + L3 * cos(q1 + q3);
        y = q2 * cos(q1) + L3 * sin(q1 + q3);
        phi_deg = q1_deg + q3_deg; 
        
        edt_x.Value = round(x, 4);
        edt_y.Value = round(y, 4);
        edt_phi.Value = round(phi_deg, 2);
        
        draw_robot(q1, q2, q3, x, y);
        update_gauges_and_jacob(q1, q2, q3);
        
        lbl_status.Text = 'Trạng thái: Điều khiển tay (Manual FK)';
        lbl_status.FontColor = 'k';
    end
    
    function run_IK_animation()
        x_target = edt_x.Value;
        y_target = edt_y.Value;
        phi_target = edt_phi.Value * pi/180;
        
        % 1. Tính IK cho điểm đích để kiểm tra tính hợp lệ trước khi chạy
        xc = x_target - L3 * cos(phi_target);
        yc = y_target - L3 * sin(phi_target);
        
        q1_cal = atan2(-xc, yc);
        q2_cal = sqrt(xc^2 + yc^2);
        q3_cal = phi_target - q1_cal;
        
        q1_deg = q1_cal * 180/pi;
        q3_deg = q3_cal * 180/pi;
        
        % Hiển thị giá trị tính toán IK ra giao diện
        lbl_ik_q1.Text = sprintf('q1: %.2f°', q1_deg);
        lbl_ik_q2.Text = sprintf('q2: %.4f m', q2_cal);
        lbl_ik_q3.Text = sprintf('q3: %.2f°', q3_deg);
        
        if q2_cal >= q2_lim(1) && q2_cal <= q2_lim(2) && ...
           q1_deg >= q1_lim(1) && q1_deg <= q1_lim(2) && ...
           q3_deg >= q3_lim(1) && q3_deg <= q3_lim(2)
           
            lbl_status.Text = 'Trạng thái: Đang chạy Animation quy hoạch quỹ đạo...';
            lbl_status.FontColor = '#D95319';
            
            % 2. Lấy vị trí hiện tại
            q1_curr = sld_q1.Value * pi/180;
            q2_curr = sld_q2.Value;
            q3_curr = sld_q3.Value * pi/180;
            
            % 3. Quy hoạch quỹ đạo trong Không gian khớp (Joint Space - PTP)
            % Phương pháp này an toàn hơn Cartesian Space vì đảm bảo 100%
            % các điểm trung gian không vi phạm giới hạn cơ khí.
            steps = 40; % Chia làm 40 khung hình mượt mà
            Q1_path = linspace(q1_curr, q1_cal, steps);
            Q2_path = linspace(q2_curr, q2_cal, steps);
            Q3_path = linspace(q3_curr, q3_cal, steps);
            
            for i = 1:steps
                q1i = Q1_path(i);
                q2i = Q2_path(i);
                q3i = Q3_path(i);
                
                % Đồng bộ lại thanh trượt (kèm hàm chặn min/max để chống sai số Matlab)
                sld_q1.Value = max(q1_lim(1), min(q1_lim(2), q1i * 180/pi));
                sld_q2.Value = max(q2_lim(1), min(q2_lim(2), q2i));
                sld_q3.Value = max(q3_lim(1), min(q3_lim(2), q3i * 180/pi));
                
                % Tính Động học thuận cho từng điểm trên quỹ đạo
                xi = -q2i * sin(q1i) + L3 * cos(q1i + q3i);
                yi = q2i * cos(q1i) + L3 * sin(q1i + q3i);
                
                % Cập nhật GUI
                draw_robot(q1i, q2i, q3i, xi, yi);
                update_gauges_and_jacob(q1i, q2i, q3i);
                
                drawnow; % Ép MATLAB vẽ lại ngay lập tức
                pause(0.04); % Độ trễ animation 40ms/frame
            end
            
            lbl_status.Text = 'Trạng thái: Đã tới mục tiêu thành công!';
            lbl_status.FontColor = '#77AC30';
        else
            lbl_status.Text = 'LỖI CƠ KHÍ: Tọa độ đích nằm ngoài vùng làm việc!';
            lbl_status.FontColor = 'r';
        end
    end
    
    function draw_robot(q1, q2, q3, x3, y3)
        x1 = 0; y1 = 0;
        x2 = -q2 * sin(q1); 
        y2 = q2 * cos(q1); 
        
        h_link2.XData = [x1 x2]; h_link2.YData = [y1 y2]; h_link2.ZData = [L1 L1];
        h_link3.XData = [x2 x3]; h_link3.YData = [y2 y3]; h_link3.ZData = [L1 L1];
        h_joints.XData = [x1 x2]; h_joints.YData = [y1 y2]; h_joints.ZData = [L1 L1];
        h_tcp.XData = x3; h_tcp.YData = y3; h_tcp.ZData = L1;
        
        % Lưu vết khâu tác động cuối (TCP Trace)
        trace_x(end+1) = x3;
        trace_y(end+1) = y3;
        trace_z(end+1) = L1;
        
        % Tránh lag máy, chỉ giữ tối đa 300 điểm trên vết
        if length(trace_x) > 300
            trace_x(1) = []; trace_y(1) = []; trace_z(1) = [];
        end
        h_trace.XData = trace_x;
        h_trace.YData = trace_y;
        h_trace.ZData = trace_z;
    end
    
    function clear_trace()
        trace_x = [];
        trace_y = [];
        trace_z = [];
        h_trace.XData = trace_x;
        h_trace.YData = trace_y;
        h_trace.ZData = trace_z;
        lbl_status.Text = 'Trạng thái: Đã xóa vết quỹ đạo.';
        lbl_status.FontColor = 'k';
    end
    
    function go_home()
        % Thiết lập tọa độ mặc định ban đầu
        edt_x.Value = 0.25;
        edt_y.Value = 0.2;
        edt_phi.Value = 0.0;
        run_IK_animation();
    end

    % Khởi động robot ở vị trí ban đầu
    run_FK();
    clear_trace(); % Clear initial jump trace
end
