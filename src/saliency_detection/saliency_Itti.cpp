//===================================================================================
// Name        : saliencyDetectionItti.cpp
// Author      : Oytun Akman, oytunakman@gmail.com
// Editor	   : Joris van de Weem, joris.vdweem@gmail.com (Conversion to
// ROS)
// Version     : 1.1
// Copyright   : Copyright (c) 2010 LGPL
// Description : C++ implementation of "A Model of Saliency-Based Visual
// Attention
//				 for Rapid Scene Analysis" by Laurent Itti, Christof
//Koch and Ernst
//				 Niebur (PAMI 1998).
//===================================================================================
// v1.1: Ported to Robot Operating System (ROS) (Joris)

#include <saliency_detection/cvgabor.h>
#include <saliency_detection/saliency_Itti.h>


void SaliencyItti::calculateSaliencyMap(const Mat &src, Mat &dst,
                                        int scaleBase) {
  createChannels(&src);

  createIntensityFeatureMaps();
  createColorFeatureMaps();
  createOrientationFeatureMaps(0);
  createOrientationFeatureMaps(2);
  createOrientationFeatureMaps(4);
  createOrientationFeatureMaps(6);

  combineFeatureMaps(scaleBase);

  resize(S, dst, src.size(), 0, 0, INTER_LINEAR);

  double minVal, maxVal;
  minMaxLoc(dst, &minVal, &maxVal, NULL, NULL);
  if (maxVal != minVal)
    dst.convertTo(dst, CV_32FC1, 1.0 / (maxVal - minVal),
                  -minVal / (maxVal - minVal));

  clearBuffers();

}

void SaliencyItti::createChannels(const Mat *src) {
  b.create(src->size(), CV_32F);
  g.create(src->size(), CV_32F);
  r.create(src->size(), CV_32F);
  I.create(src->size(), CV_32F);
  vector<Mat> planes;
  split(*src, planes);

  for (int j = 0; j < r.rows; j++)
    for (int i = 0; i < r.cols; i++) {
      b.at<float>(j, i) = planes[0].at<uchar>(j, i);
      g.at<float>(j, i) = planes[1].at<uchar>(j, i);
      r.at<float>(j, i) = planes[2].at<uchar>(j, i);
    }

  I = r + g + b;
  I = I / 3;

  normalize_rgb();
  create_RGBY();
  createScaleSpace(&I, &gaussianPyramid_I, 8);
  createScaleSpace(&R, &gaussianPyramid_R, 8);
  createScaleSpace(&G, &gaussianPyramid_G, 8);
  createScaleSpace(&B, &gaussianPyramid_B, 8);
  createScaleSpace(&Y, &gaussianPyramid_Y, 8);
}

/*r,g and b channels are normalized by I in order to decouple hue from intensity
  Because hue variations are not perceivable at very low luminance normalization
  is only applied at the locations
  where I is larger than max(I)/10 (other locations yield zero r,g,b)*/
void SaliencyItti::normalize_rgb() {
  double minVal, maxVal;
  minMaxLoc(I, &minVal, &maxVal);
  Mat mask_I;
  threshold(I, mask_I, maxVal / 10, 1.0, THRESH_BINARY);

  r = r / I;
  g = g / I;
  b = b / I;
  r = r.mul(mask_I);
  g = g.mul(mask_I);
  b = b.mul(mask_I);
}

void SaliencyItti::create_RGBY() {
  Mat gb = g + b;
  Mat rb = r + b;
  Mat rg = r + g;
  Mat rg_ = r - g;

  R = r - gb / 2;
  G = g - rb / 2;
  B = b - rg / 2;
  Y = rg / 2 - abs(rg_ / 2) - b;

  Mat mask;
  threshold(R, mask, 0.0, 1.0, THRESH_BINARY);
  R = R.mul(mask);
  threshold(G, mask, 0.0, 1.0, THRESH_BINARY);
  G = G.mul(mask);
  threshold(B, mask, 0.0, 1.0, THRESH_BINARY);
  B = B.mul(mask);
  threshold(Y, mask, 0.0, 1.0, THRESH_BINARY);
  Y = Y.mul(mask);
}

void SaliencyItti::createScaleSpace(const Mat *src, vector<Mat> *dst,
                                    int scale) {
  buildPyramid(*src, *dst, scale);
}

void SaliencyItti::createIntensityFeatureMaps() {
  int fineScaleNumber = 3;
  int scaleDiff = 2;
  int c[3] = {2, 3, 4};
  int delta[2] = {3, 4};

  for (int scaleIndex = 0; scaleIndex < fineScaleNumber; scaleIndex++) {
    Mat fineScaleImage = gaussianPyramid_I.at(c[scaleIndex]);
    Size fineScaleImageSize = fineScaleImage.size();

    for (int scaleDiffIndex = 0; scaleDiffIndex < scaleDiff; scaleDiffIndex++) {
      int s = c[scaleIndex] + delta[scaleDiffIndex];
      Mat coarseScaleImage = gaussianPyramid_I.at(s);
      Mat coarseScaleImageUp;
      resize(coarseScaleImage, coarseScaleImageUp, fineScaleImageSize, 0, 0,
             INTER_LINEAR);
      Mat I_cs = abs(fineScaleImage - coarseScaleImageUp);
      featureMaps_I.push_back(I_cs);
    }
  }
}

void SaliencyItti::createColorFeatureMaps() {
  int fineScaleNumber = 3;
  int scaleDiff = 2;
  int c[3] = {2, 3, 4};
  int delta[2] = {3, 4};

  for (int scaleIndex = 0; scaleIndex < fineScaleNumber; scaleIndex++) {
    Mat fineScaleImageR = gaussianPyramid_R.at(c[scaleIndex]);
    Mat fineScaleImageG = gaussianPyramid_G.at(c[scaleIndex]);
    Mat fineScaleImageB = gaussianPyramid_B.at(c[scaleIndex]);
    Mat fineScaleImageY = gaussianPyramid_Y.at(c[scaleIndex]);
    Size fineScaleImageSize = fineScaleImageR.size();

    Mat RGc = fineScaleImageR - fineScaleImageG;
    Mat BYc = fineScaleImageB - fineScaleImageY;

    for (int scaleDiffIndex = 0; scaleDiffIndex < scaleDiff; scaleDiffIndex++) {
      int s = c[scaleIndex] + delta[scaleDiffIndex];
      Mat coarseScaleImageR = gaussianPyramid_R.at(s);
      Mat coarseScaleImageG = gaussianPyramid_G.at(s);
      Mat coarseScaleImageB = gaussianPyramid_B.at(s);
      Mat coarseScaleImageY = gaussianPyramid_Y.at(s);

      Mat GRs = coarseScaleImageG - coarseScaleImageR;
      Mat YBs = coarseScaleImageY - coarseScaleImageB;
      Mat coarseScaleImageUpGRs, coarseScaleImageUpYBs;
      resize(GRs, coarseScaleImageUpGRs, fineScaleImageSize, 0, 0,
             INTER_LINEAR);
      resize(YBs, coarseScaleImageUpYBs, fineScaleImageSize, 0, 0,
             INTER_LINEAR);
      Mat RG_cs = abs(RGc - coarseScaleImageUpGRs);
      Mat BY_cs = abs(BYc - coarseScaleImageUpYBs);

      featureMaps_RG.push_back(RG_cs);
      featureMaps_BY.push_back(BY_cs);
    }
  }
}

void SaliencyItti::createOrientationFeatureMaps(int orientation) {
  int fineScaleNumber = 3;
  int scaleDiff = 2;
  int c[3] = {2, 3, 4};
  int delta[2] = {3, 4};
  CvGabor *gabor = new CvGabor(orientation, 0);
  IplImage *gbr_fineScaleImage, *gbr_coarseScaleImage;

  for (int scaleIndex = 0; scaleIndex < fineScaleNumber; scaleIndex++) {
    Mat fineScaleImage = gaussianPyramid_I.at(c[scaleIndex]);
    Size fineScaleImageSize = fineScaleImage.size();

    IplImage src_fineScaleImage = IplImage(fineScaleImage);
    gbr_fineScaleImage = cvCreateImage(fineScaleImage.size(), IPL_DEPTH_8U, 1);
    gabor->conv_img(&src_fineScaleImage, gbr_fineScaleImage, CV_GABOR_REAL);
    Mat src_responseImg(cv::cvarrToMat(gbr_fineScaleImage));

    for (int scaleDiffIndex = 0; scaleDiffIndex < scaleDiff; scaleDiffIndex++) {
      int s = c[scaleIndex] + delta[scaleDiffIndex];
      Mat coarseScaleImage = gaussianPyramid_I.at(s);
      IplImage src_coarseScaleImage = IplImage(coarseScaleImage);
      gbr_coarseScaleImage =
          cvCreateImage(coarseScaleImage.size(), IPL_DEPTH_8U, 1);
      gabor->conv_img(&src_coarseScaleImage, gbr_coarseScaleImage,
                      CV_GABOR_REAL);
      Mat coarse_responseImg(cv::cvarrToMat(gbr_coarseScaleImage));

      Mat coarseScaleImageUp;
      resize(coarse_responseImg, coarseScaleImageUp, fineScaleImageSize, 0, 0,
             INTER_LINEAR);

      Mat temp = abs(src_responseImg - coarseScaleImageUp);
      Mat O_cs(temp.size(), CV_32F);

      for (int j = 0; j < temp.rows; j++)
        for (int i = 0; i < temp.cols; i++)
          O_cs.at<float>(j, i) = temp.at<uchar>(j, i);

      if (orientation == 0)
        featureMaps_0.push_back(O_cs);
      if (orientation == 2)
        featureMaps_45.push_back(O_cs);
      if (orientation == 4)
        featureMaps_90.push_back(O_cs);
      if (orientation == 6)
        featureMaps_135.push_back(O_cs);

      cvReleaseImage(&gbr_coarseScaleImage);
    }
    cvReleaseImage(&gbr_fineScaleImage);
  }
}

void SaliencyItti::combineFeatureMaps(int scale) {
  Size scaleImageSize = gaussianPyramid_I.at(scale).size();
  conspicuityMap_I.create(scaleImageSize, CV_32F);
  conspicuityMap_I.setTo(0);
  conspicuityMap_C.create(scaleImageSize, CV_32F);
  conspicuityMap_C.setTo(0);
  conspicuityMap_O.create(scaleImageSize, CV_32F);
  conspicuityMap_O.setTo(0);
  Mat ori_0(scaleImageSize, CV_32F);
  ori_0.setTo(0);
  Mat ori_45(scaleImageSize, CV_32F);
  ori_45.setTo(0);
  Mat ori_90(scaleImageSize, CV_32F);
  ori_90.setTo(0);
  Mat ori_135(scaleImageSize, CV_32F);
  ori_135.setTo(0);

  Mat featureMap, featureMap_scaled, RG, BY, RG_scaled, BY_scaled;

  for (int index = 0; index < 3 * 2; index++) {
    resize(featureMaps_I.at(index), featureMap_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    mapNormalization(&featureMap_scaled);
    conspicuityMap_I = conspicuityMap_I + featureMap_scaled;

    resize(featureMaps_RG.at(index), RG_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    resize(featureMaps_BY.at(index), BY_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    mapNormalization(&RG_scaled);
    mapNormalization(&BY_scaled);
    conspicuityMap_C = conspicuityMap_C + (RG_scaled + BY_scaled);

    resize(featureMaps_0.at(index), featureMap_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    mapNormalization(&featureMap_scaled);
    ori_0 = ori_0 + featureMap_scaled;
    resize(featureMaps_45.at(index), featureMap_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    mapNormalization(&featureMap_scaled);
    ori_45 = ori_45 + featureMap_scaled;
    resize(featureMaps_90.at(index), featureMap_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    mapNormalization(&featureMap_scaled);
    ori_90 = ori_90 + featureMap_scaled;
    resize(featureMaps_135.at(index), featureMap_scaled, scaleImageSize, 0, 0,
           INTER_LINEAR);
    mapNormalization(&featureMap_scaled);
    ori_135 = ori_135 + featureMap_scaled;
  }

  mapNormalization(&ori_0);
  mapNormalization(&ori_45);
  mapNormalization(&ori_90);
  mapNormalization(&ori_135);
  conspicuityMap_O = ori_0 + ori_45 + ori_90 + ori_135;

  mapNormalization(&conspicuityMap_I);
  mapNormalization(&conspicuityMap_C);
  mapNormalization(&conspicuityMap_O);

  S = conspicuityMap_I + conspicuityMap_C + conspicuityMap_O;
  S = S / 3;
}
void SaliencyItti::mapNormalization(Mat *src) {
  double minVal, maxVal;
  minMaxLoc(*src, &minVal, &maxVal);
  *src = *src / (float)maxVal;
}

void SaliencyItti::clearBuffers() {
  gaussianPyramid_I.clear();
  gaussianPyramid_R.clear();
  gaussianPyramid_G.clear();
  gaussianPyramid_B.clear();
  gaussianPyramid_Y.clear();
  featureMaps_I.clear();
  featureMaps_RG.clear();
  featureMaps_BY.clear();
  featureMaps_0.clear();
  featureMaps_45.clear();
  featureMaps_90.clear();
  featureMaps_135.clear();
}
