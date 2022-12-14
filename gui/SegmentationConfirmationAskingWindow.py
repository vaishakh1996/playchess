# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

GUI_PKG_DIR = '/home/luca/tiago_public_ws/src/chess_gui'

class Ui_SegmentationConfirmationAskingWindow(object):
    def setupUi(self, SegmentationConfirmationAskingWindow):
        SegmentationConfirmationAskingWindow.setObjectName("SegmentationConfirmationAskingWindow")
        SegmentationConfirmationAskingWindow.resize(606, 577)
        self.verticalLayout = QtWidgets.QVBoxLayout(SegmentationConfirmationAskingWindow)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horLayoutImage = QtWidgets.QHBoxLayout()
        self.horLayoutImage.setObjectName("horLayoutImage")
        self.ImageLabel = QtWidgets.QLabel(SegmentationConfirmationAskingWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.ImageLabel.sizePolicy().hasHeightForWidth())
        self.ImageLabel.setSizePolicy(sizePolicy)
        self.ImageLabel.setMaximumSize(QtCore.QSize(500, 400))
        self.ImageLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.ImageLabel.setText("")
        self.ImageLabel.setPixmap(QtGui.QPixmap(GUI_PKG_DIR + "/images/segmentation_example.png"))
        self.ImageLabel.setScaledContents(True)
        self.ImageLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.ImageLabel.setObjectName("ImageLabel")
        self.horLayoutImage.addWidget(self.ImageLabel)
        self.verticalLayout.addLayout(self.horLayoutImage)
        self.CompletionLabel = QtWidgets.QLabel(SegmentationConfirmationAskingWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CompletionLabel.sizePolicy().hasHeightForWidth())
        self.CompletionLabel.setSizePolicy(sizePolicy)
        self.CompletionLabel.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.CompletionLabel.setFont(font)
        self.CompletionLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.CompletionLabel.setObjectName("CompletionLabel")
        self.verticalLayout.addWidget(self.CompletionLabel)
        self.line = QtWidgets.QFrame(SegmentationConfirmationAskingWindow)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout.addWidget(self.line)
        self.QuestionLabel = QtWidgets.QLabel(SegmentationConfirmationAskingWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.QuestionLabel.sizePolicy().hasHeightForWidth())
        self.QuestionLabel.setSizePolicy(sizePolicy)
        self.QuestionLabel.setMaximumSize(QtCore.QSize(16777215, 20))
        self.QuestionLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.QuestionLabel.setWordWrap(True)
        self.QuestionLabel.setObjectName("QuestionLabel")
        self.verticalLayout.addWidget(self.QuestionLabel)
        self.SuggestionLabel = QtWidgets.QLabel(SegmentationConfirmationAskingWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SuggestionLabel.sizePolicy().hasHeightForWidth())
        self.SuggestionLabel.setSizePolicy(sizePolicy)
        self.SuggestionLabel.setMaximumSize(QtCore.QSize(16777215, 20))
        font = QtGui.QFont()
        font.setPointSize(9)
        self.SuggestionLabel.setFont(font)
        self.SuggestionLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.SuggestionLabel.setWordWrap(True)
        self.SuggestionLabel.setObjectName("SuggestionLabel")
        self.verticalLayout.addWidget(self.SuggestionLabel)
        self.hor = QtWidgets.QHBoxLayout()
        self.hor.setObjectName("hor")
        self.SegmentAgainPushButton = QtWidgets.QPushButton(SegmentationConfirmationAskingWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.SegmentAgainPushButton.sizePolicy().hasHeightForWidth())
        self.SegmentAgainPushButton.setSizePolicy(sizePolicy)
        self.SegmentAgainPushButton.setMaximumSize(QtCore.QSize(200, 16777215))
        self.SegmentAgainPushButton.setObjectName("SegmentAgainPushButton")
        self.hor.addWidget(self.SegmentAgainPushButton)
        self.CorrectPushButton = QtWidgets.QPushButton(SegmentationConfirmationAskingWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.CorrectPushButton.sizePolicy().hasHeightForWidth())
        self.CorrectPushButton.setSizePolicy(sizePolicy)
        self.CorrectPushButton.setMaximumSize(QtCore.QSize(200, 16777215))
        self.CorrectPushButton.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.CorrectPushButton.setObjectName("CorrectPushButton")
        self.CorrectPushButton.setFocus() #Set the initial focus of the GUI.
        self.CorrectPushButton.setStyleSheet("background-color: rgb(176, 202, 220);")
        self.hor.addWidget(self.CorrectPushButton)
        self.verticalLayout.addLayout(self.hor)

        self.retranslateUi(SegmentationConfirmationAskingWindow)
        QtCore.QMetaObject.connectSlotsByName(SegmentationConfirmationAskingWindow)


    def retranslateUi(self, SegmentationConfirmationAskingWindow):
        _translate = QtCore.QCoreApplication.translate
        SegmentationConfirmationAskingWindow.setWindowTitle(_translate("SegmentationConfirmationAskingWindow", "Form"))
        self.CompletionLabel.setText(_translate("SegmentationConfirmationAskingWindow", "Chessboard segmentation completed!"))
        self.QuestionLabel.setText(_translate("SegmentationConfirmationAskingWindow", "Are the identified centers correctly placed around the chessboard?"))
        self.SuggestionLabel.setText(_translate("SegmentationConfirmationAskingWindow", "If not, try moving the chessboard or changing lightning in the room and perform segmetation again "))
        self.SegmentAgainPushButton.setText(_translate("SegmentationConfirmationAskingWindow", "Segment Again"))
        self.CorrectPushButton.setText(_translate("SegmentationConfirmationAskingWindow", "Correct"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    SegmentationConfirmationAskingWindow = QtWidgets.QWidget()
    ui = Ui_SegmentationConfirmationAskingWindow()
    ui.setupUi(SegmentationConfirmationAskingWindow)
    SegmentationConfirmationAskingWindow.show()
    sys.exit(app.exec_())

