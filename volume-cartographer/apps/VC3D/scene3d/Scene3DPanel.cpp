#include "Scene3DPanel.hpp"
#include "Scene3DCanvas.hpp"
#include "Scene3DMeshSource.hpp"
#include "MemoryBudgetInput.hpp"
#include "CState.hpp"
#include "VCSettings.hpp"
#include "fibers3d/Fiber3DSliceControls.hpp"
#include "fibers3d/Fiber3DSurfaceControls.hpp"
#include "vc/core/types/Volume.hpp"
#include <QCheckBox>
#include <QComboBox>
#include <QHideEvent>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QShowEvent>
#include <QToolButton>
#include <QVBoxLayout>

namespace vc3d::scene3d
{
Scene3DPanel::Scene3DPanel(CState* state, ViewerManager* viewers, QWidget* parent) : QWidget(parent)
{
    setObjectName(QStringLiteral("scene3DPanel"));
    auto* layout = new QVBoxLayout(this);
    auto* toolbar = new QHBoxLayout;
    auto button = [&](const QString& label, const char* name) {
        auto* result = new QToolButton(this);
        result->setText(label);
        result->setObjectName(QString::fromLatin1(name));
        toolbar->addWidget(result);
        return result;
    };
    auto* ct = button(tr("CT"), "scene3DCT");
    displayMode_ = new QComboBox(this);
    displayMode_->setObjectName(QStringLiteral("scene3DDisplayMode"));
    displayMode_->addItem(tr("Surface + slices"));
    displayMode_->addItem(tr("CT volume"));
    displayMode_->addItem(tr("Meshes + slices"));
    displayMode_->setToolTip(tr("Show the selected surface and CT slices, the translucent CT volume, or the stored meshes (View ▸ Meshes) with CT slices."));
    toolbar->insertWidget(0, displayMode_);
    auto* fit = button(tr("Fit all"), "scene3DFitAll");
    auto* perspective = button(tr("Perspective"), "scene3DPerspective");
    perspective->setCheckable(true);
    auto* settings = button(tr("Settings"), "scene3DSettings");
    settings->setCheckable(true);
    umbilicusButton_ = button(tr("Umbilicus"), "scene3DUmbilicus");
    umbilicusButton_->setCheckable(true);
    umbilicusButton_->setEnabled(false);   // enabled only once a View > Umbilicus selection exists
    umbilicusButton_->setToolTip(tr("Show the selected umbilicus (View ▸ Umbilicus) as a line here in 3D."));
    toolbar->addStretch();
    auto* detach = button(tr("Detach"), "scene3DDetach");
    layout->addLayout(toolbar);
    auto* volumeLabel = new QLabel(this);
    volumeLabel->setTextFormat(Qt::PlainText);
    volumeLabel->setWordWrap(true);
    layout->addWidget(volumeLabel);
    auto refreshVolume = [state, volumeLabel] {
        const auto volume = state->currentVolume();
        volumeLabel->setText(volume ? tr("CT volume: %1").arg(QString::fromStdString(volume->id())) : tr("Attach a CT volume to view its slices."));
    };
    connect(state, &CState::volumeChanged, this, refreshVolume);
    refreshVolume();

    canvas_ = new Scene3DCanvas(this);
    canvas_->setObjectName(QStringLiteral("scene3DCanvas"));
    layout->addWidget(canvas_, 1);
    meshSource_ = new Scene3DMeshSource(state, canvas_, this);
    // The mesh view reuses the one "CT texture" checkbox in the surface settings (same as "Surface + slices"),
    // routed here via the canvas so there is no duplicate control.
    connect(canvas_, &Scene3DCanvas::surfaceTexturedChanged, this, [this](bool on) { meshSource_->setTextured(on); });
    connect(canvas_, &Scene3DCanvas::surfaceTextureLevelChanged, this, [this](int level) { meshSource_->setLevel(level); });
    // "Umbilicus" toolbar toggle: 3D-view line only (the global show/hide is View > Umbilicus, fed via
    // setUmbilicusPoints). Effective only while an umbilicus is selected (button disabled otherwise).
    connect(umbilicusButton_, &QToolButton::toggled, this, [this](bool on) {
        canvas_->setUmbilicus(umbilicusPoints_);
        canvas_->setUmbilicusVisible(on && !umbilicusPoints_.empty());
    });
    slices_ = new fibers3d::Fiber3DSliceControls(state, viewers, canvas_, ct, this, QStringLiteral("scene3d"));
    slices_->enableXYPositionControl();
    connect(canvas_, &Scene3DCanvas::surfaceDisplayChanged, this, [this] {
        std::optional<Bounds> bounds;
        const int mode = displayMode_->currentIndex();
        if (const auto mesh = canvas_->surfaceMesh(); (mode == 0 || mode == 2) && mesh && !mesh->indices.empty() && canvas_->isSurfaceVisible())
            bounds = mesh->bounds;
        slices_->setMeshBounds(bounds);
    });
    layout->addWidget(slices_);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setMaximumHeight(300);
    auto* controls = new QWidget(scroll);
    auto* options = new QVBoxLayout(controls);
    auto* sources = new QHBoxLayout;
    auto* attach = new QPushButton(tr("Attach CT volume…"), controls);
    auto* import = new QPushButton(tr("Import tifxyz surface…"), controls);
    sources->addWidget(attach);
    sources->addWidget(import);
    options->addLayout(sources);
    auto* help = new QLabel(tr("While a mesh is displayed, only the XY slice is shown, centered on the mesh. Use XY slice Z to move through it, or Center on mesh to return to its center. Volumetric thickness and material settings are in Composite View. Select a surface in Main's surface list to display its mesh."), controls);
    help->setWordWrap(true);
    options->addWidget(help);
    surface_ = new fibers3d::Fiber3DSurfaceControls(state, canvas_, controls, QStringLiteral("scene3d"));
    options->addWidget(surface_);
    // In "Meshes + slices" the surface controls are suspended, so route the mesh source's progress
    // ("Preparing mesh…", "Baking CT texture…", "Mesh · N faces · CT level L") into their status label.
    connect(meshSource_, &Scene3DMeshSource::statusChanged, surface_, &fibers3d::Fiber3DSurfaceControls::setStatus);
    QSettings prefs(vc3d::settingsFilePath(), QSettings::IniFormat);
    auto* budget = makeMemoryBudgetInput(controls, prefs.value("scene3d/memoryMiB", 2048).toDouble());
    budget->setObjectName(QStringLiteral("scene3DMemoryBudgetMiB"));
    budget->setToolTip(tr("Memory allowance for 3D loading, allocated only as needed. Increase this for larger surfaces or higher-resolution CT previews. Volume resolution controls the requested preview size."));
    auto* budgetRow = new QHBoxLayout;
    auto* budgetLabel = new QLabel(tr("3D memory budget"), controls);
    budgetLabel->setBuddy(budget);
    budgetRow->addWidget(budgetLabel);
    budgetRow->addWidget(budget, 1);
    surface_->setMemoryBudget(memoryBudgetBytes(budget));
    meshSource_->setMemoryBudget(memoryBudgetBytes(budget));
    options->addLayout(budgetRow);
    scroll->setWidget(controls);
    layout->addWidget(scroll);
    // Surface sampling follows panel visibility, independently of whether its
    // settings are expanded. Slice controls remain visible while CT is enabled.
    settings->setChecked(true);
    connect(settings, &QToolButton::toggled, scroll, &QWidget::setVisible);
    connect(attach, &QPushButton::clicked, this, &Scene3DPanel::attachVolumeRequested);
    connect(import, &QPushButton::clicked, this, &Scene3DPanel::importSurfaceRequested);
    connect(detach, &QToolButton::clicked, this, &Scene3DPanel::detachRequested);
    connect(fit, &QToolButton::clicked, canvas_, &Scene3DCanvas::fitAll);
    connect(perspective, &QToolButton::toggled, canvas_, &Scene3DCanvas::setPerspective);
    perspective->setChecked(prefs.value("scene3d/perspective", false).toBool());
    connect(perspective, &QToolButton::toggled, this, [](bool enabled) {
        QSettings prefs(vc3d::settingsFilePath(), QSettings::IniFormat);
        prefs.setValue("scene3d/perspective", enabled);
    });
    connect(budget, &QDoubleSpinBox::valueChanged, this, [this, budget] {
        surface_->setMemoryBudget(memoryBudgetBytes(budget));
        meshSource_->setMemoryBudget(memoryBudgetBytes(budget));
        QSettings prefs(vc3d::settingsFilePath(), QSettings::IniFormat);
        prefs.setValue("scene3d/memoryMiB", budget->value());
    });
    connect(state, &CState::volumeClosing, canvas_, &Scene3DCanvas::clearContent);
    connect(state, &CState::volumeClosing, this, [volumeLabel] { volumeLabel->setText(tr("Attach a CT volume to view its slices.")); });
    connect(state, &CState::volumeChanged, canvas_, &Scene3DCanvas::clearContent);
    ct->setChecked(true);
    const auto surfaceHelp = help->text();
    connect(displayMode_, &QComboBox::currentIndexChanged, this, [this, ct, help, surfaceHelp, import](int index) {
        const bool volume = index == 1;
        const bool meshes = index == 2;
        if (volume) {
            savedCT_ = ct->isChecked();
            ct->setChecked(false);
            slices_->cancel();
            slices_->hide();
        } else if (!slices_->isVisible()) {
            slices_->show();   // "Surface + slices" and "Meshes + slices" both keep the slices
        }
        ct->setVisible(!volume);
        import->setVisible(index == 0);
        canvas_->clearContent();
        // The segment-surface source drives modes 0/1; the mesh source drives mode 2. Only one feeds
        // the canvas mesh at a time, so suspend the other before activating.
        meshSource_->setActive(false);
        if (meshes) {
            surface_->suspend();
            surface_->setVolumeSource(false);   // keeps the settings "CT texture" checkbox visible
            meshSource_->setTextured(canvas_->isSurfaceTextured());   // seed from the shared checkbox
            meshSource_->setLevel(canvas_->surfaceTextureLevel());    // seed the CT level from the combo
            meshSource_->setActive(true);
        } else {
            surface_->setVolumeSource(volume);
            surface_->resume();
        }
        if (!volume) ct->setChecked(savedCT_);
        help->setText(meshes ? tr("Showing stored meshes (View ▸ Meshes) in 3D with CT slices. Toggle CT texture to map scan voxels onto the mesh faces; off shows each mesh in its cross-section colour.")
                             : volume ? tr("Drag to rotate the CT volume. Volume density controls how much internal structure shows through. Increase Volume resolution for more detail; the preview stays within the memory budget and GPU limits. Choose a finer Zarr level if the source data is too coarse.")
                                      : surfaceHelp);
    });
}
void Scene3DPanel::setVisibleMeshes(const std::vector<std::string>& uuids)
{
    meshSource_->setVisibleMeshes(uuids);
}
void Scene3DPanel::setUmbilicusPoints(const std::vector<std::array<double, 3>>& points)
{
    umbilicusPoints_ = points;
    if (umbilicusButton_)
        umbilicusButton_->setEnabled(!points.empty());   // the 3D toggle only works with a selection
    canvas_->setUmbilicus(points);
    const bool show = umbilicusButton_ && umbilicusButton_->isChecked() && !points.empty();
    canvas_->setUmbilicusVisible(show);
}
void Scene3DPanel::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
    const int mode = displayMode_->currentIndex();
    if (mode != 1) slices_->resume();
    if (mode == 2) meshSource_->setActive(true);
    else surface_->resume();
}
void Scene3DPanel::hideEvent(QHideEvent* event)
{
    QWidget::hideEvent(event);
    slices_->cancel();
    surface_->suspend();
    meshSource_->setActive(false);
}
}

#ifdef VC_TESTING
#include "Scene3DPanelTest.inc"
#endif
