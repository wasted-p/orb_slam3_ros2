
#include <qevent.h>
#include <ui/pose_list.hpp>

using namespace hexapod_rviz_plugins;

PoseList::PoseList() {
  loop_start_icon_ = QIcon(":/icons/loop_start.png");
  loop_end_icon_ = QIcon(":/icons/loop_end.png");

  connect(this, &QListWidget::itemDoubleClicked, this, &PoseList::onRenamePose);
  connect(this, &QListWidget::itemClicked, this, &PoseList::onItemClicked);
  connect(this, &QListWidget::itemClicked, this, &PoseList::onPoseSelected);
  connect(this, &QListWidget::itemSelectionChanged, [=]() {
    if (this->selectedItems().isEmpty() && this->count() > 0) {
      this->setCurrentRow(0); // or reselect previous item
    }
  });
  qApp->installEventFilter(this);
}

void PoseList::removePose(size_t idx) {
  if (item(idx))
    delete item(idx);
  setCurrentRow(idx - 1);
}
void PoseList::addPose() {
  std::string name = "Pose " + std::to_string(count());
  this->addItem(name.c_str());
  this->setCurrentRow(0);
}

void PoseList::moveCurrentPose(int distance) {
  int row = currentRow();
  if (row < count() - 1) {
    QListWidgetItem *item = takeItem(row);
    insertItem(row + distance, item);
    setCurrentItem(item);
  }
}

void PoseList::moveCurrentPoseUp() { moveCurrentPose(1); }
void PoseList::moveCurrentPoseDown() { moveCurrentPose(-1); }

void PoseList::onPoseSelected() {
  size_t idx = currentRow();
  emit poseSelected(idx);
}
void PoseList::onRenamePose(QListWidgetItem *item) {
  bool ok;
  QString new_name = QInputDialog::getText(this, "Rename Pose",
                                           "Enter new name:", QLineEdit::Normal,
                                           item->text(), &ok);
  if (ok && !new_name.isEmpty()) {
    item->setText(new_name);
  }
}

void PoseList::startLoopSelection() {
  selecting_loop_ = true;
  loop_start_item_ = nullptr;
  loop_end_item_ = nullptr;
  this->setCursor(loop_cursor_);
}
void PoseList::onItemClicked(QListWidgetItem *item) {
  if (!selecting_loop_)
    return;

  if (!loop_start_item_) {
    loop_start_item_ = item;
    item->setIcon(loop_start_icon_);
  } else if (!loop_end_item_ && item != loop_start_item_) {
    loop_end_item_ = item;
    item->setIcon(loop_end_icon_);
    selecting_loop_ = false;
    this->unsetCursor();
    // Optional: emit signal or callback with loop range
  }
}

bool PoseList::eventFilter(QObject *obj, QEvent *event) {
  if (selecting_loop_ && event->type() == QEvent::MouseButtonPress) {
    QMouseEvent *mouseEvent = static_cast<QMouseEvent *>(event);
    if (!this->rect().contains(this->mapFromGlobal(mouseEvent->globalPos()))) {
      cancelLoopSelection();
    }
  }
  return QWidget::eventFilter(obj, event);
}
void PoseList::cancelLoopSelection() {
  selecting_loop_ = false;
  this->unsetCursor();

  if (loop_start_item_)
    loop_start_item_->setIcon(QIcon());
  if (loop_end_item_)
    loop_end_item_->setIcon(QIcon());

  loop_start_item_ = nullptr;
  loop_end_item_ = nullptr;
}
