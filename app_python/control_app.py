import sys
import re
import time
import threading
import queue

import serial
import serial.tools.list_ports
import numpy as np

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg


# ============================================================
# CONFIGURACIÓN
# ============================================================

UART_BAUDRATE = 115200

# El firmware trabaja con una muestra cada 20 ms
CONTROL_PERIOD = 0.020       # s
CONTROL_FREQUENCY = 1.0 / CONTROL_PERIOD

# 500 muestras × 20 ms = 10 segundos
WINDOW_SAMPLES = 500
WINDOW_SECONDS = WINDOW_SAMPLES * CONTROL_PERIOD

# El firmware utiliza:
# PWM_RES = LEDC_TIMER_10_BIT
# PWM_MAX = (1 << PWM_RES) = 1024
PWM_MAX = 1024.0

UART_READ_TIMEOUT = 0.10

# Cola máxima de líneas recibidas.
# La aplicación consume normalmente mucho más rápido de lo que llegan.
UART_QUEUE_SIZE = 1000


# ============================================================
# EXPRESIONES REGULARES DEL PROTOCOLO
# ============================================================

FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

# Telemetría del firmware:
#
# R90.00A87.50U512.00
#
# R = referencia
# A = ángulo
# U = control
DATA_RE = re.compile(
    rf"^R({FLOAT_RE})A({FLOAT_RE})U({FLOAT_RE})$"
)

# Respuesta de parámetros:
#
# ok kp:2.500000
# ok ki:0.100000
# ok kd:0.050000
# ok windup:1.000000
# ok kick:0.000000
# ok ref:90.000000
PARAM_ACK_RE = re.compile(
    rf"^ok\s+(kp|ki|kd|windup|kick|ref):({FLOAT_RE})$",
    re.IGNORECASE
)

# Respuestas de estado:
#
# ok start
# ok stop
STATE_ACK_RE = re.compile(
    r"^ok\s+(start|stop)$",
    re.IGNORECASE
)


# ============================================================
# HILO DE RECEPCIÓN UART
# ============================================================

class UARTReader(threading.Thread):
    """
    Hilo dedicado exclusivamente a leer el puerto UART.

    No actualiza widgets.
    No modifica arrays de la GUI.
    No interpreta parámetros.

    Su única responsabilidad es:
        UART -> líneas completas -> Queue
    """

    def __init__(self, ser, rx_queue):
        super().__init__(daemon=True)

        self.ser = ser
        self.rx_queue = rx_queue
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():

            try:
                line = self.ser.readline()

                if not line:
                    continue

                text = line.decode("utf-8", errors="replace").strip()

                if not text:
                    continue

                self._push_line(text)

            except serial.SerialException as exc:
                if not self.stop_event.is_set():
                    self._push_line(f"__SERIAL_ERROR__:{exc}")
                break

            except Exception as exc:
                if not self.stop_event.is_set():
                    self._push_line(f"__THREAD_ERROR__:{exc}")
                break

    def _push_line(self, line):
        """
        Inserta una línea en la cola.

        Si la cola estuviera llena, descartamos la línea más vieja.
        Esto evita que el hilo UART se bloquee indefinidamente.
        """

        try:
            self.rx_queue.put_nowait(line)

        except queue.Full:
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self.rx_queue.put_nowait(line)
            except queue.Full:
                pass

    def stop(self):
        self.stop_event.set()


# ============================================================
# VENTANA PRINCIPAL
# ============================================================

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Control de Posición PID")
        self.resize(1150, 800)

        # ----------------------------------------------------
        # UART
        # ----------------------------------------------------

        self.ser = serial.Serial()

        # Toda comunicación recibida entra aquí.
        self.rx_queue = queue.Queue(maxsize=UART_QUEUE_SIZE)

        self.reader_thread = None

        self.connected = False
        self.streaming = False

        # ----------------------------------------------------
        # ESTADO RECIBIDO DEL FIRMWARE
        # ----------------------------------------------------

        self.last_ref = 0.0
        self.last_angle = 0.0
        self.last_error = 0.0

        # U que llega del firmware está en PWM:
        # aproximadamente -1024 ... +1024
        self.last_u_raw = 0.0

        # La GUI lo representa en porcentaje.
        self.last_u_percent = 0.0

        # Parámetros confirmados por el firmware
        self.kp = None
        self.ki = None
        self.kd = None
        self.windup = None
        self.kick = None
        self.firmware_ref = None

        # ----------------------------------------------------
        # DATOS DEL GRÁFICO
        # ----------------------------------------------------

        self.plot_ref = np.full(WINDOW_SAMPLES, np.nan)
        self.plot_ang = np.full(WINDOW_SAMPLES, np.nan)
        self.plot_err = np.full(WINDOW_SAMPLES, np.nan)
        self.plot_u = np.full(WINDOW_SAMPLES, np.nan)

        self.x_axis = (
            np.arange(WINDOW_SAMPLES) * CONTROL_PERIOD
        )

        # ----------------------------------------------------
        # INTERFAZ
        # ----------------------------------------------------

        self.central_stack = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.central_stack)

        self.init_menu_ui()
        self.init_plot_ui()

        # ----------------------------------------------------
        # TIMER DE LA GUI
        # ----------------------------------------------------

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.process_uart_and_update_gui)
        self.timer.start(20)

    # ========================================================
    # PANTALLA DE CONEXIÓN
    # ========================================================

    def init_menu_ui(self):

        self.menu_widget = QtWidgets.QWidget()

        layout = QtWidgets.QVBoxLayout(self.menu_widget)
        layout.setContentsMargins(50, 50, 50, 50)
        layout.setSpacing(15)

        title = QtWidgets.QLabel("Configuración de puerto serie")
        title.setFont(
            QtGui.QFont(
                "Arial",
                16,
                QtGui.QFont.Bold
            )
        )

        layout.addWidget(title)

        form = QtWidgets.QFormLayout()

        self.combo_port = QtWidgets.QComboBox()
        self.refresh_ports()

        self.combo_baud = QtWidgets.QComboBox()
        self.combo_baud.addItems([
            "115200",
            "230400",
            "460800",
            "921600"
        ])
        self.combo_baud.setCurrentText("115200")

        form.addRow("Puerto:", self.combo_port)
        form.addRow("Baud Rate:", self.combo_baud)

        layout.addLayout(form)

        self.btn_refresh = QtWidgets.QPushButton(
            "Refrescar Puertos"
        )

        self.btn_refresh.clicked.connect(
            self.refresh_ports
        )

        layout.addWidget(self.btn_refresh)

        self.btn_start_init = QtWidgets.QPushButton(
            "Conectar"
        )

        self.btn_start_init.setFixedHeight(45)

        self.btn_start_init.setStyleSheet(
            "background-color: #2ecc71;"
            "color: white;"
            "font-weight: bold;"
        )

        self.btn_start_init.clicked.connect(
            self.start_connection
        )

        layout.addWidget(self.btn_start_init)

        self.lbl_connection_status = QtWidgets.QLabel(
            "Desconectado"
        )

        self.lbl_connection_status.setAlignment(
            QtCore.Qt.AlignCenter
        )

        layout.addWidget(
            self.lbl_connection_status
        )

        layout.addStretch()

        self.central_stack.addWidget(
            self.menu_widget
        )

    # ========================================================
    # PANTALLA DE CONTROL
    # ========================================================

    def init_plot_ui(self):

        self.plot_container = QtWidgets.QWidget()

        main_layout = QtWidgets.QVBoxLayout(
            self.plot_container
        )

        # ----------------------------------------------------
        # GRÁFICO
        # ----------------------------------------------------

        self.plot = pg.PlotWidget()

        self.plot.setBackground("k")

        self.plot.showGrid(
            x=True,
            y=True,
            alpha=0.3
        )

        self.plot.setLabel(
            "bottom",
            "Tiempo",
            units="s"
        )

        self.plot.setLabel(
            "left",
            "Ángulo / Error",
            units="°"
        )

        self.plot.setXRange(
            0,
            WINDOW_SECONDS,
            padding=0
        )

        self.plot.setYRange(
            -360,
            360,
            padding=0
        )

        self.plot.setLimits(
            xMin=0,
            xMax=WINDOW_SECONDS,
            yMin=-360,
            yMax=360
        )

        self.plot.showAxis("right")

        # ----------------------------------------------------
        # LEYENDA
        # ----------------------------------------------------

        self.legend = self.plot.addLegend(
            offset=(10, 10)
        )

        # ----------------------------------------------------
        # CURVAS PRINCIPALES
        # ----------------------------------------------------

        self.curve_ref = self.plot.plot(
            self.x_axis,
            self.plot_ref,
            pen=pg.mkPen(
                "y",
                width=2,
                style=QtCore.Qt.DashLine
            ),
            name="Referencia [°]"
        )

        self.curve_ang = self.plot.plot(
            self.x_axis,
            self.plot_ang,
            pen=pg.mkPen(
                "b",
                width=2
            ),
            name="Ángulo [°]"
        )

        self.curve_err = self.plot.plot(
            self.x_axis,
            self.plot_err,
            pen=pg.mkPen(
                "r",
                width=2
            ),
            name="Error [°]"
        )

        # ----------------------------------------------------
        # SEGUNDO EJE PARA U
        # ----------------------------------------------------

        self.p2 = pg.ViewBox()

        self.plot.scene().addItem(self.p2)

        self.plot.getAxis("right").linkToView(
            self.p2
        )

        self.p2.setXLink(
            self.plot
        )

        self.plot.getAxis("right").setLabel(
            "Señal de control",
            units="%"
        )

        self.curve_u = pg.PlotCurveItem(
            self.x_axis,
            self.plot_u,
            pen=pg.mkPen(
                "g",
                width=2
            )
        )

        self.p2.addItem(
            self.curve_u
        )

        self.p2.setYRange(
            -100,
            100,
            padding=0
        )

        self.p2.setLimits(
            yMin=-100,
            yMax=100
        )

        self.legend.addItem(
            self.curve_u,
            "Señal U [%]"
        )

        self.plot.getViewBox().sigResized.connect(
            self.update_views
        )

        main_layout.addWidget(
            self.plot,
            stretch=3
        )

        # ----------------------------------------------------
        # PANELES INFERIORES
        # ----------------------------------------------------

        bottom_layout = QtWidgets.QHBoxLayout()

        # ====================================================
        # LECTURAS
        # ====================================================

        group_leidas = QtWidgets.QGroupBox(
            "Lectura"
        )

        layout_leidas = QtWidgets.QGridLayout(
            group_leidas
        )

        self.lbl_read_ref = QtWidgets.QLabel(
            "Referencia: --"
        )

        self.lbl_read_ang = QtWidgets.QLabel(
            "Ángulo: --"
        )

        self.lbl_read_err = QtWidgets.QLabel(
            "Error: --"
        )

        self.lbl_read_u = QtWidgets.QLabel(
            "Señal U: --"
        )

        self.lbl_param_kp = QtWidgets.QLabel(
            "Kp: --"
        )

        self.lbl_param_ki = QtWidgets.QLabel(
            "Ki: --"
        )

        self.lbl_param_kd = QtWidgets.QLabel(
            "Kd: --"
        )

        self.lbl_param_wu = QtWidgets.QLabel(
            "Anti Wind-Up: --"
        )

        self.lbl_param_ks = QtWidgets.QLabel(
            "Kickstart: --"
        )

        labels_left = [
            self.lbl_read_ref,
            self.lbl_read_ang,
            self.lbl_read_err,
            self.lbl_read_u
        ]

        labels_right = [
            self.lbl_param_kp,
            self.lbl_param_ki,
            self.lbl_param_kd,
            self.lbl_param_wu,
            self.lbl_param_ks
        ]

        for i, label in enumerate(labels_left):

            label.setFont(
                QtGui.QFont(
                    "Consolas",
                    13,
                    QtGui.QFont.Bold
                )
            )

            layout_leidas.addWidget(
                label,
                i,
                0
            )

        for i, label in enumerate(labels_right):

            label.setFont(
                QtGui.QFont(
                    "Consolas",
                    13,
                    QtGui.QFont.Bold
                )
            )

            layout_leidas.addWidget(
                label,
                i,
                1
            )

        bottom_layout.addWidget(
            group_leidas,
            stretch=1
        )

        # ====================================================
        # ESCRITURA
        # ====================================================

        group_mandar = QtWidgets.QGroupBox(
            "Configuración"
        )

        layout_mandar = QtWidgets.QGridLayout(
            group_mandar
        )

        # ---------------- Referencia ----------------

        self.ref_input = self.create_spinbox(
            "Ref: ",
            0,
            360,
            1.0
        )

        btn_ref = QtWidgets.QPushButton(
            "Enviar Ref"
        )

        btn_ref.clicked.connect(
            lambda:
            self.send_parameter(
                "ref",
                self.ref_input.value()
            )
        )

        # ---------------- Kp ----------------

        self.kp_input = self.create_spinbox(
            "Kp: ",
            0,
            1000,
            0.1
        )

        btn_kp = QtWidgets.QPushButton(
            "Enviar Kp"
        )

        btn_kp.clicked.connect(
            lambda:
            self.send_parameter(
                "kp",
                self.kp_input.value()
            )
        )

        # ---------------- Ki ----------------

        self.ki_input = self.create_spinbox(
            "Ki: ",
            0,
            1000,
            0.1
        )

        btn_ki = QtWidgets.QPushButton(
            "Enviar Ki"
        )

        btn_ki.clicked.connect(
            lambda:
            self.send_parameter(
                "ki",
                self.ki_input.value()
            )
        )

        # ---------------- Kd ----------------

        self.kd_input = self.create_spinbox(
            "Kd: ",
            0,
            1000,
            0.1
        )

        btn_kd = QtWidgets.QPushButton(
            "Enviar Kd"
        )

        btn_kd.clicked.connect(
            lambda:
            self.send_parameter(
                "kd",
                self.kd_input.value()
            )
        )

        # ---------------- Windup ----------------

        self.chk_windup = QtWidgets.QCheckBox(
            "Activar Anti Wind-Up"
        )

        self.chk_windup.setChecked(True)

        btn_windup = QtWidgets.QPushButton(
            "Enviar"
        )

        btn_windup.clicked.connect(
            lambda:
            self.send_parameter(
                "windup",
                1 if self.chk_windup.isChecked()
                else 0
            )
        )

        # ---------------- Kick ----------------

        self.chk_kick = QtWidgets.QCheckBox(
            "Activar Kickstart"
        )

        self.chk_kick.setChecked(False)

        btn_kick = QtWidgets.QPushButton(
            "Enviar"
        )

        btn_kick.clicked.connect(
            lambda:
            self.send_parameter(
                "kick",
                1 if self.chk_kick.isChecked()
                else 0
            )
        )

        # ---------------- Layout ----------------

        layout_mandar.addWidget(
            self.ref_input,
            0,
            0
        )

        layout_mandar.addWidget(
            btn_ref,
            0,
            1
        )

        layout_mandar.addWidget(
            self.kp_input,
            1,
            0
        )

        layout_mandar.addWidget(
            btn_kp,
            1,
            1
        )

        layout_mandar.addWidget(
            self.ki_input,
            2,
            0
        )

        layout_mandar.addWidget(
            btn_ki,
            2,
            1
        )

        layout_mandar.addWidget(
            self.kd_input,
            3,
            0
        )

        layout_mandar.addWidget(
            btn_kd,
            3,
            1
        )

        layout_mandar.addWidget(
            self.chk_windup,
            4,
            0
        )

        layout_mandar.addWidget(
            btn_windup,
            4,
            1
        )

        layout_mandar.addWidget(
            self.chk_kick,
            5,
            0
        )

        layout_mandar.addWidget(
            btn_kick,
            5,
            1
        )

        # ---------------- Todos ----------------

        self.btn_send_all = QtWidgets.QPushButton(
            "Enviar todos los parámetros"
        )

        self.btn_send_all.setStyleSheet(
            "background-color: #3498db;"
            "color: white;"
            "font-weight: bold;"
        )

        self.btn_send_all.clicked.connect(
            self.send_all_parameters
        )

        layout_mandar.addWidget(
            self.btn_send_all,
            6,
            0,
            1,
            2
        )

        # ---------------- Start/Stop ----------------

        self.btn_start_stop = QtWidgets.QPushButton(
            "Iniciar Control"
        )

        self.btn_start_stop.clicked.connect(
            self.toggle_start_stop
        )

        layout_mandar.addWidget(
            self.btn_start_stop,
            7,
            0
        )

        # ---------------- Disconnect ----------------

        self.btn_reset = QtWidgets.QPushButton(
            "Desconectar"
        )

        self.btn_reset.setStyleSheet(
            "background-color: #e74c3c;"
            "color: white;"
            "font-weight: bold;"
        )

        self.btn_reset.clicked.connect(
            self.reset_app
        )

        layout_mandar.addWidget(
            self.btn_reset,
            7,
            1
        )

        # ---------------- Estado ----------------

        self.lbl_status = QtWidgets.QLabel(
            "Conectado - Control detenido"
        )

        self.lbl_status.setAlignment(
            QtCore.Qt.AlignCenter
        )

        layout_mandar.addWidget(
            self.lbl_status,
            8,
            0,
            1,
            2
        )

        bottom_layout.addWidget(
            group_mandar,
            stretch=1
        )

        main_layout.addLayout(
            bottom_layout,
            stretch=1
        )

        self.central_stack.addWidget(
            self.plot_container
        )

    # ========================================================
    # UTILIDADES DE GUI
    # ========================================================

    def create_spinbox(
        self,
        prefix,
        minimum,
        maximum,
        step
    ):

        spinbox = QtWidgets.QDoubleSpinBox()

        spinbox.setRange(
            minimum,
            maximum
        )

        spinbox.setSingleStep(
            step
        )

        spinbox.setPrefix(
            prefix
        )

        spinbox.setDecimals(
            3
        )

        spinbox.setFont(
            QtGui.QFont(
                "Arial",
                11
            )
        )

        return spinbox

    def update_views(self):

        self.p2.setGeometry(
            self.plot.getViewBox().sceneBoundingRect()
        )

        self.p2.linkedViewChanged(
            self.plot.getViewBox(),
            self.p2.XAxis
        )

    # ========================================================
    # PUERTOS
    # ========================================================

    def refresh_ports(self):

        current = self.combo_port.currentText()

        ports = [
            port.device
            for port in serial.tools.list_ports.comports()
        ]

        self.combo_port.clear()
        self.combo_port.addItems(ports)

        if current in ports:
            self.combo_port.setCurrentText(
                current
            )

    # ========================================================
    # CONEXIÓN
    # ========================================================

    def start_connection(self):

        if self.connected:
            return

        port = self.combo_port.currentText().strip()

        if not port:
            QtWidgets.QMessageBox.warning(
                self,
                "Puerto serie",
                "No hay ningún puerto seleccionado."
            )
            return

        try:

            self.ser.port = port

            self.ser.baudrate = int(
                self.combo_baud.currentText()
            )

            self.ser.bytesize = serial.EIGHTBITS
            self.ser.parity = serial.PARITY_NONE
            self.ser.stopbits = serial.STOPBITS_ONE
            self.ser.timeout = UART_READ_TIMEOUT
            self.ser.write_timeout = 1.0

            self.ser.open()

            # Algunas placas ESP pueden reiniciarse al abrir el puerto.
            time.sleep(0.2)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.connected = True
            self.streaming = False

            self.clear_runtime_data()

            # Hilo UART
            self.reader_thread = UARTReader(
                self.ser,
                self.rx_queue
            )

            self.reader_thread.start()

            self.central_stack.setCurrentIndex(1)

            self.btn_start_stop.setText(
                "Iniciar Control"
            )

            self.lbl_status.setText(
                "Conectado - Control detenido"
            )

            self.lbl_connection_status.setText(
                f"Conectado a {port}"
            )

        except Exception as exc:

            self.connected = False
            self.streaming = False

            if self.ser.is_open:
                self.ser.close()

            QtWidgets.QMessageBox.critical(
                self,
                "Error de conexión",
                f"No se pudo abrir el puerto:\n\n{exc}"
            )

    # ========================================================
    # DESCONEXIÓN
    # ========================================================

    def reset_app(self):

        self.timer.stop()

        # Intentar detener el control antes de cerrar UART.
        if self.connected and self.ser.is_open:

            try:
                self.send_command("stop")
                self.ser.flush()
            except Exception:
                pass

        self.streaming = False
        self.connected = False

        # Detener hilo RX
        if self.reader_thread is not None:

            self.reader_thread.stop()

        # Cerrar el puerto.
        #
        # Esto además libera un readline() bloqueado.
        if self.ser.is_open:

            try:
                self.ser.close()
            except Exception:
                pass

        # Esperar al hilo.
        if self.reader_thread is not None:

            self.reader_thread.join(
                timeout=0.5
            )

            self.reader_thread = None

        # Limpiar cola restante.
        self.clear_rx_queue()

        self.central_stack.setCurrentIndex(0)

        self.lbl_connection_status.setText(
            "Desconectado"
        )

    # ========================================================
    # ENVÍO UART
    # ========================================================

    def send_command(self, command):

        if not self.connected or not self.ser.is_open:
            return False

        try:

            message = (
                command.rstrip("\r\n")
                + "\n"
            )

            self.ser.write(
                message.encode("ascii")
            )

            return True

        except (
            serial.SerialException,
            OSError
        ) as exc:

            self.handle_serial_error(
                str(exc)
            )

            return False

    def send_parameter(
        self,
        parameter,
        value
    ):

        command = (
            f"set {parameter} {value:.6f}"
        )

        self.send_command(command)

    def send_all_parameters(self):

        if not self.connected:
            return

        self.send_parameter(
            "ref",
            self.ref_input.value()
        )

        self.send_parameter(
            "kp",
            self.kp_input.value()
        )

        self.send_parameter(
            "ki",
            self.ki_input.value()
        )

        self.send_parameter(
            "kd",
            self.kd_input.value()
        )

        self.send_parameter(
            "windup",
            1 if self.chk_windup.isChecked()
            else 0
        )

        self.send_parameter(
            "kick",
            1 if self.chk_kick.isChecked()
            else 0
        )

    # ========================================================
    # START / STOP
    # ========================================================

    def toggle_start_stop(self):

        if not self.connected:
            return

        if self.streaming:

            if self.send_command("stop"):

                # Estado provisional.
                # El ACK también será procesado.
                self.streaming = False

                self.btn_start_stop.setText(
                    "Iniciar Control"
                )

                self.lbl_status.setText(
                    "Control detenido"
                )

        else:

            if self.send_command("start"):

                self.streaming = True

                self.btn_start_stop.setText(
                    "Detener Control"
                )

                self.lbl_status.setText(
                    "Control iniciado"
                )

    # ========================================================
    # PROCESAMIENTO DE UART
    # ========================================================

    def process_uart_and_update_gui(self):

        if not self.connected:
            return

        lines_processed = 0

        # Procesar las líneas que hayan llegado desde
        # el último tick del QTimer.
        while lines_processed < 1000:

            try:
                line = self.rx_queue.get_nowait()

            except queue.Empty:
                break

            lines_processed += 1

            self.process_uart_line(
                line
            )

        # Actualizar GUI
        self.update_labels()
        self.update_plot()

    def process_uart_line(self, line):

        # ----------------------------------------------------
        # ERROR DE UART
        # ----------------------------------------------------

        if line.startswith(
            "__SERIAL_ERROR__:"
        ):

            error = line.split(
                ":",
                1
            )[1]

            self.handle_serial_error(
                error
            )

            return

        if line.startswith(
            "__THREAD_ERROR__:"
        ):

            error = line.split(
                ":",
                1
            )[1]

            self.handle_serial_error(
                error
            )

            return

        # ----------------------------------------------------
        # ACK START / STOP
        # ----------------------------------------------------

        state_match = STATE_ACK_RE.match(
            line
        )

        if state_match:

            state = (
                state_match.group(1)
                .lower()
            )

            if state == "start":

                self.streaming = True

                self.btn_start_stop.setText(
                    "Detener Control"
                )

                self.lbl_status.setText(
                    "Control activo"
                )

            elif state == "stop":

                self.streaming = False

                self.btn_start_stop.setText(
                    "Iniciar Control"
                )

                self.lbl_status.setText(
                    "Control detenido"
                )

            return

        # ----------------------------------------------------
        # ACK DE PARÁMETROS
        # ----------------------------------------------------

        param_match = PARAM_ACK_RE.match(
            line
        )

        if param_match:

            parameter = (
                param_match.group(1)
                .lower()
            )

            value = float(
                param_match.group(2)
            )

            self.update_parameter(
                parameter,
                value
            )

            return

        # ----------------------------------------------------
        # TELEMETRÍA
        # ----------------------------------------------------

        data_match = DATA_RE.match(
            line
        )

        if data_match:

            ref = float(
                data_match.group(1)
            )

            angle = float(
                data_match.group(2)
            )

            u_raw = float(
                data_match.group(3)
            )

            error = self.calculate_angular_error(
                ref,
                angle
            )

            # El firmware manda U en PWM.
            # Convertimos a porcentaje solamente
            # para representación gráfica.
            u_percent = (
                u_raw
                / PWM_MAX
                * 100.0
            )

            # Protección adicional para el gráfico.
            u_percent = float(
                np.clip(
                    u_percent,
                    -100.0,
                    100.0
                )
            )

            self.rx_data_pending.append(
                (
                    ref,
                    angle,
                    error,
                    u_percent
                )
            )

            self.last_ref = ref
            self.last_angle = angle
            self.last_error = error
            self.last_u_raw = u_raw
            self.last_u_percent = u_percent

    # ========================================================
    # PARÁMETROS
    # ========================================================

    def update_parameter(
        self,
        parameter,
        value
    ):

        if parameter == "kp":

            self.kp = value

            self.kp_input.setValue(
                value
            )

        elif parameter == "ki":

            self.ki = value

            self.ki_input.setValue(
                value
            )

        elif parameter == "kd":

            self.kd = value

            self.kd_input.setValue(
                value
            )

        elif parameter == "windup":

            self.windup = (
                value != 0
            )

            self.chk_windup.setChecked(
                self.windup
            )

        elif parameter == "kick":

            self.kick = (
                value != 0
            )

            self.chk_kick.setChecked(
                self.kick
            )

        elif parameter == "ref":

            self.firmware_ref = value

            self.ref_input.setValue(
                value
            )

    # ========================================================
    # CÁLCULO DE ERROR ANGULAR
    # ========================================================

    @staticmethod
    def calculate_angular_error(
        ref,
        angle
    ):

        error = ref - angle

        # Misma lógica utilizada por el firmware.
        if error > 180.0:

            error -= 360.0

        elif error < -180.0:

            error += 360.0

        return error

    # ========================================================
    # ACTUALIZACIÓN DE LABELS
    # ========================================================

    def update_labels(self):

        self.lbl_read_ref.setText(
            f"Referencia: {self.last_ref:.2f} °"
        )

        self.lbl_read_ang.setText(
            f"Ángulo: {self.last_angle:.2f} °"
        )

        self.lbl_read_err.setText(
            f"Error: {self.last_error:.2f} °"
        )

        self.lbl_read_u.setText(
            f"Señal U: {self.last_u_percent:.2f} %"
        )

        if self.kp is not None:

            self.lbl_param_kp.setText(
                f"Kp: {self.kp:.3f}"
            )

        if self.ki is not None:

            self.lbl_param_ki.setText(
                f"Ki: {self.ki:.3f}"
            )

        if self.kd is not None:

            self.lbl_param_kd.setText(
                f"Kd: {self.kd:.3f}"
            )

        if self.windup is not None:

            state = (
                "SÍ"
                if self.windup
                else "NO"
            )

            self.lbl_param_wu.setText(
                f"Anti Wind-Up: {state}"
            )

        if self.kick is not None:

            state = (
                "SÍ"
                if self.kick
                else "NO"
            )

            self.lbl_param_ks.setText(
                f"Kickstart: {state}"
            )

    # ========================================================
    # GRÁFICO
    # ========================================================

    def update_plot(self):

        if not hasattr(
            self,
            "rx_data_pending"
        ):
            self.rx_data_pending = []

        if not self.rx_data_pending:
            return

        samples = self.rx_data_pending
        self.rx_data_pending = []

        num_new = len(samples)

        refs = np.asarray(
            [sample[0] for sample in samples],
            dtype=float
        )

        angles = np.asarray(
            [sample[1] for sample in samples],
            dtype=float
        )

        errors = np.asarray(
            [sample[2] for sample in samples],
            dtype=float
        )

        controls = np.asarray(
            [sample[3] for sample in samples],
            dtype=float
        )

        # ----------------------------------------------------
        # Si entraron más muestras que el histórico,
        # conservamos solamente las últimas.
        # ----------------------------------------------------

        if num_new >= WINDOW_SAMPLES:

            self.plot_ref[:] = refs[
                -WINDOW_SAMPLES:
            ]

            self.plot_ang[:] = angles[
                -WINDOW_SAMPLES:
            ]

            self.plot_err[:] = errors[
                -WINDOW_SAMPLES:
            ]

            self.plot_u[:] = controls[
                -WINDOW_SAMPLES:
            ]

        else:

            self.plot_ref = np.roll(
                self.plot_ref,
                -num_new
            )

            self.plot_ang = np.roll(
                self.plot_ang,
                -num_new
            )

            self.plot_err = np.roll(
                self.plot_err,
                -num_new
            )

            self.plot_u = np.roll(
                self.plot_u,
                -num_new
            )

            self.plot_ref[
                -num_new:
            ] = refs

            self.plot_ang[
                -num_new:
            ] = angles

            self.plot_err[
                -num_new:
            ] = errors

            self.plot_u[
                -num_new:
            ] = controls

        # ----------------------------------------------------
        # Actualizar curvas
        # ----------------------------------------------------

        self.curve_ref.setData(
            self.x_axis,
            self.plot_ref
        )

        self.curve_ang.setData(
            self.x_axis,
            self.plot_ang
        )

        self.curve_err.setData(
            self.x_axis,
            self.plot_err
        )

        self.curve_u.setData(
            self.x_axis,
            self.plot_u
        )

    # ========================================================
    # LIMPIEZA
    # ========================================================

    def clear_rx_queue(self):

        while True:

            try:
                self.rx_queue.get_nowait()

            except queue.Empty:
                break

    def clear_runtime_data(self):

        self.clear_rx_queue()

        self.rx_data_pending = []

        self.plot_ref.fill(
            np.nan
        )

        self.plot_ang.fill(
            np.nan
        )

        self.plot_err.fill(
            np.nan
        )

        self.plot_u.fill(
            np.nan
        )

        self.last_ref = 0.0
        self.last_angle = 0.0
        self.last_error = 0.0
        self.last_u_raw = 0.0
        self.last_u_percent = 0.0

        self.kp = None
        self.ki = None
        self.kd = None
        self.windup = None
        self.kick = None
        self.firmware_ref = None

        self.update_labels()

        self.curve_ref.setData(
            self.x_axis,
            self.plot_ref
        )

        self.curve_ang.setData(
            self.x_axis,
            self.plot_ang
        )

        self.curve_err.setData(
            self.x_axis,
            self.plot_err
        )

        self.curve_u.setData(
            self.x_axis,
            self.plot_u
        )

    # ========================================================
    # ERROR DE SERIAL
    # ========================================================

    def handle_serial_error(
        self,
        message
    ):

        self.connected = False
        self.streaming = False

        self.lbl_status.setText(
            "Error de comunicación"
        )

        print(
            f"Error UART: {message}"
        )

    # ========================================================
    # CIERRE DE VENTANA
    # ========================================================

    def closeEvent(
        self,
        event
    ):

        self.reset_app()

        event.accept()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    app = QtWidgets.QApplication(
        sys.argv
    )

    app.setStyle(
        "Fusion"
    )

    window = MainWindow()
    window.show()

    sys.exit(
        app.exec_()
    )