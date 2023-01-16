#pragma once
#include <atlstr.h>
#include <iostream>
#include "image.h"


namespace read_image {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for Form1
	/// </summary>


	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::MenuStrip^ menuStrip1;
	protected:
	private: System::Windows::Forms::ToolStripMenuItem^ fileToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^ openToolStripMenuItem;
	private: System::Windows::Forms::PictureBox^ pictureBox1;
	private: System::Windows::Forms::OpenFileDialog^ openFileDialog1;
	private: System::Windows::Forms::Button^ CalibButton;
	private: System::Windows::Forms::RichTextBox^ richTextBox1;
	private: System::Windows::Forms::Button^ WorldCoordEnter;
	private: System::Windows::Forms::TextBox^ xCoordtextB;
	private: System::Windows::Forms::TextBox^ zCoordtextB;
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::TextBox^ yCoordtextB;
	private: System::Windows::Forms::Button^ calcButton;
	private: System::Windows::Forms::RichTextBox^ richTextBox2;
	private: System::Windows::Forms::RadioButton^ AutoRadioButton;
	private: System::Windows::Forms::RadioButton^ ManuelRadioButton;
	private: System::Windows::Forms::ToolStripMenuItem^ showEdgesToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^ showLinesToolStripMenuItem;
	private: System::Windows::Forms::Button^ measureButton;

	private: System::Windows::Forms::CheckBox^ checkBox1;

	private:
		System::ComponentModel::Container^ components;

		/* ----------------------------------------Variables for camera calibration-----------------------------------------*/
		int numofDetectedP = 0;
		point* DetectedPoints = new point[numofDetectedP]; // Line intersection points

		point* imagepts = new point[9];
		int clikCount = -1;
		float* worldManuelP = new float[27];// Manuelly entered world point xyz -> 3 dim * 6 pts = 18

		point* autoDetectP = new point[6]; // auto detected points

		float* Projection = new float[12]; // projection matrix

		point* p = new point; //input point

		bool isCalib = false;

		bool measureMod = false;
		int measurePCount = 0;

	private: System::Windows::Forms::RadioButton^ manuelGetFile;
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::OpenFileDialog^ openFileDialog2;

		   float* imageCoord = new float[2]; // for measure point 
		
	 /* -----------------------------------------------------------------------------------------------------------------*/
#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void ShowRGBImages(image im, System::Windows::Forms::PictureBox^ pictureBox) {
			pictureBox1->Width = im.w;
			pictureBox1->Height = im.h;
			pictureBox1->Refresh();
			Bitmap^ surface = gcnew Bitmap(im.w, im.h);
			pictureBox->Image = surface;
			Color c;
			int psw, bufpos;
			psw = im.w * 3;
			for (int row = 0; row < im.h; row++)
				for (int col = 0; col < im.w; col++) {
					bufpos = row * psw + col * im.c;
					c = Color::FromArgb(im.data[bufpos], im.data[bufpos + 1], im.data[bufpos + 2]);
					surface->SetPixel(col, row, c);
				}
		}//ShowImages
		void ShowGrayImages(image im, System::Windows::Forms::PictureBox^ pictureBox) {
			pictureBox1->Width = im.w;
			pictureBox1->Height = im.h;
			pictureBox1->Refresh();
			Bitmap^ surface = gcnew Bitmap(im.w, im.h);
			pictureBox->Image = surface;
			Color c;
			int psw, bufpos;
			psw = im.w;
			for (int row = 0; row < im.h; row++)
				for (int col = 0; col < im.w; col++) {
					bufpos = row * psw + col;
					c = Color::FromArgb(im.data[bufpos], im.data[bufpos], im.data[bufpos]);
					surface->SetPixel(col, row, c);
				}
		}//ShowImages
		void InitializeComponent(void)
		{
			this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
			this->fileToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->showEdgesToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->showLinesToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->openFileDialog1 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->CalibButton = (gcnew System::Windows::Forms::Button());
			this->richTextBox1 = (gcnew System::Windows::Forms::RichTextBox());
			this->WorldCoordEnter = (gcnew System::Windows::Forms::Button());
			this->xCoordtextB = (gcnew System::Windows::Forms::TextBox());
			this->zCoordtextB = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->yCoordtextB = (gcnew System::Windows::Forms::TextBox());
			this->calcButton = (gcnew System::Windows::Forms::Button());
			this->richTextBox2 = (gcnew System::Windows::Forms::RichTextBox());
			this->AutoRadioButton = (gcnew System::Windows::Forms::RadioButton());
			this->ManuelRadioButton = (gcnew System::Windows::Forms::RadioButton());
			this->measureButton = (gcnew System::Windows::Forms::Button());
			this->checkBox1 = (gcnew System::Windows::Forms::CheckBox());
			this->manuelGetFile = (gcnew System::Windows::Forms::RadioButton());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->openFileDialog2 = (gcnew System::Windows::Forms::OpenFileDialog());
			this->menuStrip1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->SuspendLayout();
			// 
			// menuStrip1
			// 
			this->menuStrip1->ImageScalingSize = System::Drawing::Size(20, 20);
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->fileToolStripMenuItem });
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Padding = System::Windows::Forms::Padding(8, 2, 0, 2);
			this->menuStrip1->Size = System::Drawing::Size(1377, 28);
			this->menuStrip1->TabIndex = 0;
			this->menuStrip1->Text = L"menuStrip1";
			// 
			// fileToolStripMenuItem
			// 
			this->fileToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {
				this->openToolStripMenuItem,
					this->showEdgesToolStripMenuItem, this->showLinesToolStripMenuItem
			});
			this->fileToolStripMenuItem->Name = L"fileToolStripMenuItem";
			this->fileToolStripMenuItem->Size = System::Drawing::Size(46, 24);
			this->fileToolStripMenuItem->Text = L"File";
			// 
			// openToolStripMenuItem
			// 
			this->openToolStripMenuItem->Name = L"openToolStripMenuItem";
			this->openToolStripMenuItem->Size = System::Drawing::Size(172, 26);
			this->openToolStripMenuItem->Text = L"Open";
			this->openToolStripMenuItem->Click += gcnew System::EventHandler(this, &Form1::openToolStripMenuItem_Click);
			// 
			// showEdgesToolStripMenuItem
			// 
			this->showEdgesToolStripMenuItem->Name = L"showEdgesToolStripMenuItem";
			this->showEdgesToolStripMenuItem->Size = System::Drawing::Size(172, 26);
			this->showEdgesToolStripMenuItem->Text = L"Show Edges";
			this->showEdgesToolStripMenuItem->Click += gcnew System::EventHandler(this, &Form1::showEdgesToolStripMenuItem_Click);
			// 
			// showLinesToolStripMenuItem
			// 
			this->showLinesToolStripMenuItem->Name = L"showLinesToolStripMenuItem";
			this->showLinesToolStripMenuItem->Size = System::Drawing::Size(172, 26);
			this->showLinesToolStripMenuItem->Text = L"Show Lines";
			this->showLinesToolStripMenuItem->Click += gcnew System::EventHandler(this, &Form1::showLinesToolStripMenuItem_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->Location = System::Drawing::Point(17, 36);
			this->pictureBox1->Margin = System::Windows::Forms::Padding(4);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(709, 686);
			this->pictureBox1->TabIndex = 1;
			this->pictureBox1->TabStop = false;
			this->pictureBox1->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &Form1::pictureBox1_MouseClick);
			// 
			// openFileDialog1
			// 
			this->openFileDialog1->FileName = L"openFileDialog1";
			// 
			// CalibButton
			// 
			this->CalibButton->Location = System::Drawing::Point(819, 248);
			this->CalibButton->Name = L"CalibButton";
			this->CalibButton->Size = System::Drawing::Size(128, 60);
			this->CalibButton->TabIndex = 2;
			this->CalibButton->Text = L"Calibration";
			this->CalibButton->UseVisualStyleBackColor = true;
			this->CalibButton->Click += gcnew System::EventHandler(this, &Form1::CalibButton_Click);
			// 
			// richTextBox1
			// 
			this->richTextBox1->Location = System::Drawing::Point(760, 333);
			this->richTextBox1->Name = L"richTextBox1";
			this->richTextBox1->Size = System::Drawing::Size(279, 252);
			this->richTextBox1->TabIndex = 3;
			this->richTextBox1->Text = L"";
			// 
			// WorldCoordEnter
			// 
			this->WorldCoordEnter->Location = System::Drawing::Point(1180, 142);
			this->WorldCoordEnter->Name = L"WorldCoordEnter";
			this->WorldCoordEnter->Size = System::Drawing::Size(99, 40);
			this->WorldCoordEnter->TabIndex = 4;
			this->WorldCoordEnter->Text = L"Enter";
			this->WorldCoordEnter->UseVisualStyleBackColor = true;
			this->WorldCoordEnter->Click += gcnew System::EventHandler(this, &Form1::WorldCoordEnter_Click);
			// 
			// xCoordtextB
			// 
			this->xCoordtextB->Location = System::Drawing::Point(1146, 102);
			this->xCoordtextB->Name = L"xCoordtextB";
			this->xCoordtextB->Size = System::Drawing::Size(35, 22);
			this->xCoordtextB->TabIndex = 5;
			// 
			// zCoordtextB
			// 
			this->zCoordtextB->Location = System::Drawing::Point(1275, 102);
			this->zCoordtextB->Name = L"zCoordtextB";
			this->zCoordtextB->Size = System::Drawing::Size(35, 22);
			this->zCoordtextB->TabIndex = 6;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(1143, 65);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(167, 34);
			this->label1->TabIndex = 8;
			this->label1->Text = L"Enter World Coordinates \r\n X :              Y:             Z:";
			// 
			// yCoordtextB
			// 
			this->yCoordtextB->Location = System::Drawing::Point(1212, 102);
			this->yCoordtextB->Name = L"yCoordtextB";
			this->yCoordtextB->Size = System::Drawing::Size(35, 22);
			this->yCoordtextB->TabIndex = 9;
			// 
			// calcButton
			// 
			this->calcButton->Location = System::Drawing::Point(1062, 248);
			this->calcButton->Name = L"calcButton";
			this->calcButton->Size = System::Drawing::Size(128, 60);
			this->calcButton->TabIndex = 10;
			this->calcButton->Text = L"Calculate World\r\n Coordinate";
			this->calcButton->UseVisualStyleBackColor = true;
			this->calcButton->Click += gcnew System::EventHandler(this, &Form1::calcButton_Click);
			// 
			// richTextBox2
			// 
			this->richTextBox2->Location = System::Drawing::Point(1079, 333);
			this->richTextBox2->Name = L"richTextBox2";
			this->richTextBox2->Size = System::Drawing::Size(231, 252);
			this->richTextBox2->TabIndex = 11;
			this->richTextBox2->Text = L"";
			// 
			// AutoRadioButton
			// 
			this->AutoRadioButton->AutoSize = true;
			this->AutoRadioButton->Checked = true;
			this->AutoRadioButton->Location = System::Drawing::Point(775, 78);
			this->AutoRadioButton->Name = L"AutoRadioButton";
			this->AutoRadioButton->Size = System::Drawing::Size(58, 21);
			this->AutoRadioButton->TabIndex = 12;
			this->AutoRadioButton->TabStop = true;
			this->AutoRadioButton->Text = L"Auto";
			this->AutoRadioButton->UseVisualStyleBackColor = true;
			// 
			// ManuelRadioButton
			// 
			this->ManuelRadioButton->AutoSize = true;
			this->ManuelRadioButton->Location = System::Drawing::Point(854, 78);
			this->ManuelRadioButton->Name = L"ManuelRadioButton";
			this->ManuelRadioButton->Size = System::Drawing::Size(75, 21);
			this->ManuelRadioButton->TabIndex = 13;
			this->ManuelRadioButton->TabStop = true;
			this->ManuelRadioButton->Text = L"Manuel";
			this->ManuelRadioButton->UseVisualStyleBackColor = true;
			// 
			// measureButton
			// 
			this->measureButton->Location = System::Drawing::Point(1215, 248);
			this->measureButton->Name = L"measureButton";
			this->measureButton->Size = System::Drawing::Size(128, 57);
			this->measureButton->TabIndex = 14;
			this->measureButton->Text = L"Measure a Distance";
			this->measureButton->UseVisualStyleBackColor = true;
			this->measureButton->Click += gcnew System::EventHandler(this, &Form1::measureButton_Click);
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Location = System::Drawing::Point(1215, 221);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(116, 21);
			this->checkBox1->TabIndex = 15;
			this->checkBox1->Text = L"Measure Mod";
			this->checkBox1->UseVisualStyleBackColor = true;
			this->checkBox1->CheckedChanged += gcnew System::EventHandler(this, &Form1::checkBox1_CheckedChanged);
			// 
			// manuelGetFile
			// 
			this->manuelGetFile->AutoSize = true;
			this->manuelGetFile->Location = System::Drawing::Point(955, 78);
			this->manuelGetFile->Name = L"manuelGetFile";
			this->manuelGetFile->Size = System::Drawing::Size(137, 21);
			this->manuelGetFile->TabIndex = 16;
			this->manuelGetFile->TabStop = true;
			this->manuelGetFile->Text = L"Manuel From File";
			this->manuelGetFile->UseVisualStyleBackColor = true;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(763, 590);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(89, 17);
			this->label2->TabIndex = 17;
			this->label2->Text = L"Point Count :";
			// 
			// openFileDialog2
			// 
			this->openFileDialog2->FileName = L"openFileDialog2";
	
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(8, 16);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1377, 735);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->manuelGetFile);
			this->Controls->Add(this->checkBox1);
			this->Controls->Add(this->measureButton);
			this->Controls->Add(this->ManuelRadioButton);
			this->Controls->Add(this->AutoRadioButton);
			this->Controls->Add(this->richTextBox2);
			this->Controls->Add(this->calcButton);
			this->Controls->Add(this->yCoordtextB);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->zCoordtextB);
			this->Controls->Add(this->xCoordtextB);
			this->Controls->Add(this->WorldCoordEnter);
			this->Controls->Add(this->richTextBox1);
			this->Controls->Add(this->CalibButton);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->menuStrip1);
			this->MainMenuStrip = this->menuStrip1;
			this->Margin = System::Windows::Forms::Padding(4);
			this->Name = L"Form1";
			this->Text = L"Form1";
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}

#pragma endregion


		void openToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e) {
			CString str;
			if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK) {
				str = openFileDialog1->FileName;
				CStringA s2(str);
				const char* input = s2;
				image im = load_image(input);
				image grayim = RgbToGray(im);						// gray image
				image smoothim = Smoothing(grayim, 3);				// smoothing
				gradient grad = Gradient(smoothim, grad);			//gradient of smooth image	//this kullan
				image edgeim = CannyEdge(grad);						// Edge detection
				hough HoughSpace = HoughLine(edgeim, HoughSpace);		//Hough Space of edge image //this kullan
				int numoflines = 0;
				line* Lines;
				DrawSelectedLines(HoughSpace, im, numoflines, Lines);			// Line Detection
				point* Points; // detected points local
				int numofpoints = 0;
				IntersectionPoint(numoflines, Lines, HoughSpace, Points, numofpoints, im); 
				// Line Intersection Points
				numofDetectedP = numofpoints;
				DetectedPoints = new point[numofDetectedP];									// Global pointer
				DetectedPoints = Points;									// Global pointer
				if (AutoRadioButton->Checked){
					point* autoDetectPoints = new point[6]; // auto calibration point local
					isCalib = autoCalibVerticalImage(DetectedPoints, numofDetectedP, autoDetectPoints, im); //Auto Calibration
					if (isCalib == true) {
						////autoDetectP = autoDetectPoints;									// auto calibration point Global Pointer
						//for (int i = 0; i < 6; i++) {
						//	autoDetectP[i].id = autoDetectPoints[i].id;
						//	autoDetectP[i].Y = autoDetectPoints[i].Y;
						//	autoDetectP[i].X = autoDetectPoints[i].X;
						//}
						FILE* worldFile;
						worldFile = fopen("calibration world points.txt", "r");
						rewind(worldFile);
						int count = 6;
						float* world = new float[count * 3];
						for (int i = 0; i < count; i++) {
							fscanf(worldFile, "%f", &world[i * 3 + 0]);
							fscanf(worldFile, "%f", &world[i * 3 + 1]);
							fscanf(worldFile, "%f", &world[i * 3 + 2]);
						}
						Compute_Projection_Matrix(count, world, autoDetectPoints, Projection);
						for (int i = 0; i < 3; i++)
							richTextBox1->AppendText(Projection[i * 4] + "\t" + Projection[i * 4 + 1] 
								+ "\t" + Projection[i * 4 + 2] + "\t" + Projection[i * 4 + 3] + "\n");
						MessageBox::Show("Calibration completed, Click on image to calculate cliked point's world coords!");
					}
					else {
						System::Windows::Forms::MessageBox::Show("Auto Calibration points couldn't detected.");
						AutoRadioButton->Checked = false;
						ManuelRadioButton->Checked = true;
					}
				}
				ShowRGBImages(im, pictureBox1);
			}//
		}//openTool
	private: System::Void showEdgesToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e) {
		CString str;

		if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK) {
			//pictureBox1->ImageLocation = openFileDialog1->FileName;
			str = openFileDialog1->FileName;
			CStringA s2(str);
			const char* input = s2;
			image im = load_image(input);
			image grayim = RgbToGray(im);						// gray image

			image smoothim = Smoothing(grayim, 3);				// smoothing

			gradient grad = Gradient(smoothim, grad);			//gradient of smooth image	//this kullan

			image edgeim = CannyEdge(grad);						// Edge detection
			ShowGrayImages(edgeim, pictureBox1);
		}
	}
	private: System::Void showLinesToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e) {
		CString str;

		if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK) {
			//pictureBox1->ImageLocation = openFileDialog1->FileName;
			str = openFileDialog1->FileName;
			CStringA s2(str);
			const char* input = s2;
			image im = load_image(input);
			image grayim = RgbToGray(im);						// gray image

			image smoothim = Smoothing(grayim, 3);				// smoothing

			gradient grad = Gradient(smoothim, grad);			//gradient of smooth image	//this kullan

			image edgeim = CannyEdge(grad);						// Edge detection

			hough HoughSpace = HoughLine(edgeim, HoughSpace);		//Hough Space of edge image //this kullan

			int numoflines = 0;
			line* Lines;

			DrawSelectedLines(HoughSpace, im, numoflines, Lines);			// Line Detection
			ShowRGBImages(im, pictureBox1);
		}
	}
	private: System::Void pictureBox1_MouseClick(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {


		if (ManuelRadioButton->Checked && !isCalib) { // For Manuel Calibration
			int xArea = 5;
			int yArea = 5;
			bool found = false;
			for (int i = 0; i < numofDetectedP; i++)
				if (abs(e->X - DetectedPoints[i].X) <= xArea && abs(e->Y - DetectedPoints[i].Y) <= yArea)// mouse ile alýnan noktayý tespit edilen noktalar ile kontrol et
				{
					clikCount++;
					std::cout << clikCount << " : point" << "x : " << DetectedPoints[i].X << "---   y : " << DetectedPoints[i].Y << "---  id : " << DetectedPoints[i].id << "\n";
					//-------------------image pts arrayi her giriþ aldýðýnda güncelle
					imagepts[clikCount].X = DetectedPoints[i].X;
					imagepts[clikCount].Y = DetectedPoints[i].Y;
					label2->Text = "Point Count : " + Convert::ToString(clikCount + 1);

					MessageBox::Show("Enter World Coordinates");

					found = true;
					break;

				}
			if (!found)System::Windows::Forms::MessageBox::Show("Point doesn't Find in image");


		}
		if (manuelGetFile->Checked && !isCalib) { // For Manuel Calibration
			int xArea = 5;
			int yArea = 5;
			bool found = false;
			for (int i = 0; i < numofDetectedP; i++)
				if (abs(e->X - DetectedPoints[i].X) <= xArea && abs(e->Y - DetectedPoints[i].Y) <= yArea)
				// mouse ile alýnan noktayý tespit edilen noktalar ile kontrol et
				{
					clikCount++;
					imagepts[clikCount].X = DetectedPoints[i].X;
					imagepts[clikCount].Y = DetectedPoints[i].Y;
					found = true;
					label2->Text = "Point Count : " + Convert::ToString(clikCount+1);
					break;
				}
			if (!found)System::Windows::Forms::MessageBox::Show("Point doesn't Find in image");
		}

		if (measureMod) // For measure a two point
		{
			imageCoord[measurePCount * 2] = e->X - 3; //input
			imageCoord[measurePCount * 2 + 1] = e->Y - 3;
			measurePCount++;
			if (measurePCount == 2) {
				MessageBox::Show("Click on Measure button");
			}

			//richTextBox2->AppendText("X : " + realWorld[0] + "Y : " + realWorld[1] + "\n");
		}
		
		p->X = e->X;// For Reconstruct Points
		p->Y = e->Y;
	}
	private: System::Void WorldCoordEnter_Click(System::Object^ sender, System::EventArgs^ e) {
		worldManuelP[clikCount * 3 + 0] = Convert::ToInt32(xCoordtextB->Text);
		worldManuelP[clikCount * 3 + 1] = Convert::ToInt32(yCoordtextB->Text);
		worldManuelP[clikCount * 3 + 2] = Convert::ToInt32(zCoordtextB->Text);
		System::Windows::Forms::MessageBox::Show("Done");

	}

	private: System::Void CalibButton_Click(System::Object^ sender, System::EventArgs^ e) {
		/*if (AutoRadioButton->Checked == true) {
			/*FILE* worldFile;
			worldFile = fopen("calibration world points.txt", "r");
			rewind(worldFile);
			int count = 6;
			float* world = new float[count * 3];
			for (int i = 0; i < count; i++) {
				fscanf(worldFile, "%f", &world[i * 3 + 0]);
				fscanf(worldFile, "%f", &world[i * 3 + 1]);
				fscanf(worldFile, "%f", &world[i * 3 + 2]);
			}

			Compute_Projection_Matrix(count, world, autoDetectP, Projection);
			for (int i = 0; i < 3; i++)
			{
				richTextBox1->AppendText(Projection[i * 4] + "\t" + Projection[i * 4 + 1] + "\t" + Projection[i * 4 + 2] + "\t" + Projection[i * 4 + 3] + "\n");
			}
			MessageBox::Show("Calibration completed, Click on image to calculate cliked point's world coords!");

			isCalib = true;
		}*/
		if (ManuelRadioButton->Checked) {
			Compute_Projection_Matrix(9, worldManuelP, imagepts, Projection);
			for (int i = 0; i < 3; i++)
			{
				richTextBox1->AppendText(Projection[i * 4] + "\t" + Projection[i * 4 + 1] + "\t" + Projection[i * 4 + 2] + "\t" + Projection[i * 4 + 3] + "\n");
			}
			MessageBox::Show("Calibration completed, Click on image to calculate cliked point's world coords!");

			isCalib = true;
		}
		else if(manuelGetFile->Checked){
			CString str;
			int count = clikCount + 1;
			float* world = new float[count * 3];
			if (openFileDialog2->ShowDialog() == System::Windows::Forms::DialogResult::OK) {

				FILE* worldFile2;
				str = openFileDialog2->FileName;
				CStringA s2(str);
				const char* input2 = s2;
				worldFile2 = fopen(input2, "r");
				rewind(worldFile2);
				for (int i = 0; i < count; i++) {
					fscanf(worldFile2, "%f", &world[i * 3 + 0]);
					fscanf(worldFile2, "%f", &world[i * 3 + 1]);
					fscanf(worldFile2, "%f", &world[i * 3 + 2]);
				}
			}
			Compute_Projection_Matrix(count,world, imagepts, Projection);
			for (int i = 0; i < 3; i++)
			{
				richTextBox1->AppendText(Projection[i * 4] + "\t" + Projection[i * 4 + 1] + "\t" + Projection[i * 4 + 2] + "\t" + Projection[i * 4 + 3] + "\n");
			}
			MessageBox::Show("Calibration completed, Click on image to calculate cliked point's world coords!");

			isCalib = true;
		}
	}
	private: System::Void calcButton_Click(System::Object^ sender, System::EventArgs^ e) {

		float* worldCoord = new float[2];
		worldCoord[0] = p->X - 3; //input
		worldCoord[1] = p->Y - 3;

		float* realWorld = new float[3];

		Reconstruct(1, worldCoord, Projection, realWorld);
		richTextBox2->AppendText("X : " + realWorld[0] + "Y : " + realWorld[1] + "\n");
	}


	private: System::Void checkBox1_CheckedChanged(System::Object^ sender, System::EventArgs^ e) {
		if (checkBox1->Checked) {

			if (!isCalib)
				MessageBox::Show("You have to calibrate first");
			else {
				measureMod = true;
				MessageBox::Show("Clik on two point on image");
			}
		}
	}

	private: System::Void measureButton_Click(System::Object^ sender, System::EventArgs^ e) {
		if (measurePCount == 2)
		{
			float* realWorld = new float[6]; // for measrue point
			Reconstruct(2, imageCoord, Projection, realWorld);
			float distance = sqrt(pow(realWorld[0] - realWorld[3], 2) + pow(realWorld[1] - realWorld[4], 2));
			richTextBox2->AppendText("X : " + realWorld[0] + "Y : " + realWorld[1] + "\n");
			richTextBox2->AppendText("X : " + realWorld[3] + "Y : " + realWorld[4] + "\n");
			richTextBox2->AppendText("Distance : " + distance + " cm" +"\n");
			measurePCount = 0;
		}
	}
	
};
}
