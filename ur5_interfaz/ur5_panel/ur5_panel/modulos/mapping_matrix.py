from PyQt5.QtWidgets import *


class MappingMatrixMixin:
    """
    Matriz de mapeo de ejes (que eje del joystick/haptico mueve que eje del
    robot, y con que signo). Es un widget generico (_create_mapping_matrix)
    reutilizado 4 veces por ui_controller_config.py: r1_linear, r1_rot,
    r2_linear, r2_rot. Todo el estado vive en self._mapping_matrices[key],
    no en atributos sueltos; _on_mapping_clicked solo mantiene la matriz
    consistente -- quien realmente escribe en r{id}_control_config son los
    callbacks on_linear_mapping_changed/on_rot_mapping_changed/
    on_*_invert_changed, en ui_controller_config.py.
    """

    def _ensure_mapping_storage(self):
        """Crea self._mapping_matrices (dict key -> estado del widget) la
        primera vez que se llama; _create_mapping_matrix lo llena, y
        _on_mapping_clicked/on_*_mapping_changed/on_*_invert_changed lo leen."""
        if not hasattr(self, "_mapping_matrices"):
            self._mapping_matrices = {}

    def _create_mapping_matrix(
        self,
        *,
        key: str,
        title: str,
        row_labels,
        col_labels,
        initial_selection=None,
        invert_label: str = "-1",
        parent=None,
    ) -> QGroupBox:
        """Crea una matriz 3x3 (o NxM) de QRadioButton con selección única por columnas.

        La regla es la misma que en a.py: cada fila debe tener una columna distinta.
        Se usa `clicked` para evitar bucles al hacer `setChecked()` programáticamente.
        """
        self._ensure_mapping_storage()

        widget = QGroupBox(title, parent)
        layout = QGridLayout()
        widget.setLayout(layout)

        row_labels = list(row_labels)
        col_labels = list(col_labels)

        # Cabeceras
        layout.addWidget(QLabel(), 0, 0)
        layout.addWidget(QLabel(invert_label), 0, 1)
        for j, label in enumerate(col_labels):
            layout.addWidget(QLabel(label), 0, 2 + j)
        for i, label in enumerate(row_labels):
            layout.addWidget(QLabel(label), 2 + i, 0)

        invert_checks = [QCheckBox(widget) for _ in row_labels]
        for i, cb in enumerate(invert_checks):
            layout.addWidget(cb, 2 + i, 1)

        groups = [QButtonGroup(widget) for _ in row_labels]
        radios = [[QRadioButton(widget) for _ in col_labels] for _ in row_labels]

        if initial_selection is None:
            if len(col_labels) == 0:
                selection = [0 for _ in row_labels]
            else:
                selection = [i % len(col_labels) for i in range(len(row_labels))]
        else:
            selection = list(initial_selection)

        for row_idx in range(len(row_labels)):
            for col_idx in range(len(col_labels)):
                rb = radios[row_idx][col_idx]
                groups[row_idx].addButton(rb, col_idx)
                layout.addWidget(rb, 2 + row_idx, 2 + col_idx)
                if selection[row_idx] == col_idx:
                    rb.setChecked(True)
                rb.clicked.connect(
                    lambda checked, r=row_idx, c=col_idx, k=key: self._on_mapping_clicked(k, r, c)
                )

        self._mapping_matrices[key] = {
            "widget": widget,
            "layout": layout,
            "invert_checks": invert_checks,
            "groups": groups,
            "radios": radios,
            "selection": selection,
            "row_labels": row_labels,
            "col_labels": col_labels,
        }
        return widget

    def _on_mapping_clicked(self, key: str, fila_clicada: int, nueva_columna: int) -> None:
        """Handler de bajo nivel de un click en la matriz: solo mantiene la
        invariante "una columna por fila" (si otra fila ya usaba esa columna,
        la mueve a la columna que dejó libre la fila clickeada). NO actualiza
        r{id}_control_config -- eso lo hacen on_linear_mapping_changed/
        on_rot_mapping_changed, conectados aparte en setup_control_config_connections."""
        data = self._mapping_matrices.get(key)
        if not data:
            return

        selection = data["selection"]
        radios = data["radios"]

        columna_antigua = selection[fila_clicada]
        if columna_antigua == nueva_columna:
            return

        fila_en_conflicto = -1
        for r, col in enumerate(selection):
            if r != fila_clicada and col == nueva_columna:
                fila_en_conflicto = r
                break

        if fila_en_conflicto != -1:
            # setChecked() NO dispara 'clicked' => sin recursión.
            radios[fila_en_conflicto][columna_antigua].setChecked(True)
            selection[fila_en_conflicto] = columna_antigua

        selection[fila_clicada] = nueva_columna
