
#include <hexapod_rviz_panels/motion_editor/ui/pose_list.hpp>
#include <qevent.h>
#include <qobjectdefs.h>

using namespace hexapod_rviz_plugins;

PoseList::PoseList() {

  connect(this, &QListWidget::itemDoubleClicked, this, &PoseList::onRenamePose);
  connect(this, &QListWidget::itemClicked, this, &PoseList::onItemClicked);
  connect(this, &QListWidget::itemSelectionChanged, [=]() {
    if (this->selectedItems().isEmpty() && this->count() > 0) {
      this->setCurrentRow(0); // or reselect previous item
    }
  });
}

void PoseList::removePose(size_t idx) {
  // if (idx < 0)
  if (item(idx))
    delete item(idx);
  if (idx > 0)
    setCurrentRow(idx - 1);
}

void PoseList::moveCurrentPose(int distance) {
  int row = currentRow();
  if (row + distance >= count() || row + distance < 0)
    return;

  QListWidgetItem *item = takeItem(row);
  insertItem(row + distance, item);
  setCurrentItem(item);
  emit poseMoved(row, row + distance);
}

void PoseList::moveCurrentPoseUp() { moveCurrentPose(-1); }
void PoseList::moveCurrentPoseDown() { moveCurrentPose(1); }

void PoseList::onPoseSelected() {}
void PoseList::onRenamePose(QListWidgetItem *item) {
  bool ok;
  QString new_name = QInputDialog::getText(this, "Rename Pose",
                                           "Enter new name:", QLineEdit::Normal,
                                           item->text(), &ok);
  if (ok && !new_name.isEmpty()) {
    item->setText(new_name);
  }
}

void PoseList::onItemClicked(QListWidgetItem *item) {
  size_t idx = currentRow();
  emit poseSelected(idx);
  return;
}
