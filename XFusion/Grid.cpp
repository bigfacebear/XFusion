#include "Grid.h"

Grid::Grid(int K):K(K) {
	voxels = new Voxel**[K];
	for (int i = 0; i < K; i++) {
		voxels[i] = new Voxel*[K];
		for (int j = 0; j < K; j++) {
			voxels[i][j] = new Voxel[K];
		}
	}

	for (int i = 0; i < K; i++) {
		for (int j = 0; j < K; j++) {
			for (int k = 0; k < K; k++) {
				voxels[i][j][k].C = cv::Vec3b(255, 0, 0);
				voxels[i][j][k].S = 0.1;
			}
		}
	}

	for (int i = K / 4; i < K * 3 / 4; i++) {
		for (int j = K / 4; j < K * 3 / 4; j++) {
			for (int k = K / 4; k < K * 3 / 4; k++) {
				voxels[i][j][k].C = cv::Vec3b(255, 0, 0);
				voxels[i][j][k].S = -0.1;
			}
		}
	}

}

Grid::~Grid() {
	for (int i = 0; i < K; i++) {
		for (int j = 0; j < K; j++) {
			delete[] voxels[i][j];
		}
		delete[] voxels[i];
	}
	delete[] voxels;
}

inline Voxel * Grid::getVoxelByCoordinate(double x, double y, double z) {
	if (x >= K || y >= K || z >= K || x < 0 || y < 0 || z < 0) {
		return nullptr;
	}
	else {
		return &voxels[(int)x][(int)y][(int)z];
	}
}

inline Voxel * Grid::getVoxelByCoordinate(cv::Vec3d v) {
	return getVoxelByCoordinate(v[0], v[1], v[2]);
}

bool Grid::getColorAndPointByRay(cv::Vec3d p, cv::Vec3d v, cv::Vec3b & C, cv::Mat & worldP) {
	p[0] += K / 2; // move the grid corner to the original point of world's coordinate
	p[1] += K / 2;

	// the direction to traverse layers of grid. dir = 0 for x, dir = 1 for y, dir = 2 for z
	int dir = 0;
	int beg = 0;
	int step = 1;
	if (fabs(v[1]) > fabs(v[0]) && fabs(v[1]) > fabs(v[2])) {
		dir = 1;
	}
	else if (fabs(v[2]) > fabs(v[0]) && fabs(v[2]) > fabs(v[1])) {
		dir = 2;
	}
	if (v[dir] < 0) {
		beg = K - 1;
		step = -1;
	}
	int dir1 = (dir + 1) % 3, dir2 = (dir + 2) % 3;

	cv::Vec3b old_C, new_C;
	double old_S = INFINITY, new_S;
	cv::Mat old_real_p, new_real_p;
	for (int i = beg; i >= 0 && i < K; i++) {
		double t;
		cv::Vec3d p0, p1, p2;
		Voxel* voxel[3] = { nullptr, nullptr, nullptr };
		t = (i - p[dir]) / v[dir];
		if (t < 0) {
			continue;
		}
		p0[dir] = (double)i;
		p0[dir1] = p[dir1] + t * v[dir1];
		p0[dir2] = p[dir2] + t * v[dir2];
		voxel[0] = getVoxelByCoordinate(p0);
		t = (i + 1 - p[dir]) / v[dir];
		if (t < 0) {
			continue;
		}
		p2[dir] = (double)i;
		p2[dir1] = p[dir1] + t * v[dir1];
		p2[dir2] = p[dir2] + t * v[dir2];
		voxel[2] = getVoxelByCoordinate(p2);
		if (min(p0[dir1], p2[dir1]) - (int)max(p0[dir1], p2[dir1]) < 0) {
			if (min(p0[dir2], p2[dir2]) - (int)max(p0[dir2], p2[dir2]) < 0) {
				p1[dir] = i;
				p1[dir1] = (v[dir1] > 0) ? min(p0[dir1], p2[dir1]) : max(p0[dir1], p2[dir1]);
				p1[dir2] = (v[dir2] > 0) ? min(p0[dir2], p2[dir2]) : max(p0[dir2], p2[dir2]);
				voxel[1] = getVoxelByCoordinate(p1);
			}
		}
		int cnt = 0;
		new_S = 0;
		new_C = cv::Vec3b::all(0);
		for (int j = 0; j < 3; j++) {
			if (voxel[j]) {
				new_C = (double)cnt / (cnt + 1) * new_C + (double)1 / (cnt + 1) * voxel[j]->C;
				new_S = (double)cnt / (cnt + 1) * new_S + (double)1 / (cnt + 1) * voxel[j]->S;
				cnt++;
			}
		}
		if (cnt == 0) {
			continue;
		}
		cv::Vec3d tmp;
		tmp[dir] = i + 0.5;
		tmp[dir1] = (p0[dir1] + p2[dir1]) / 2;
		tmp[dir2] = (p2[dir2] + p2[dir2]) / 2;
		new_real_p = cv::Mat(tmp);
		if (new_S <= 0) {
			if (new_S == 0) {
				C = new_C;
			}
			else {
				double tmp_newS = -1 / new_S, tmp_oldS = 1 / old_S;
				double tmp_sumS = tmp_newS + tmp_oldS;
				double para_new = tmp_newS / tmp_sumS;
				double para_old = tmp_oldS / tmp_sumS;
				C = para_new * new_C + para_new * old_C;
				new_real_p = para_new * new_real_p + para_old * old_real_p;
			}
			worldP = new_real_p - cv::Mat(cv::Vec3d(K / 2, K / 2, 0));
			return true;
		}
		else {
			old_C = new_C;
			old_S = new_S;
			old_real_p = new_real_p;
		}
	}
	return false;
}


//bool Grid::getColorAndPointByRay(cv::Vec3d p, cv::Vec3d v, cv::Vec3b & C, cv::Mat & worldP) {
//	p[0] += K / 2; // move the grid corner to the original point of world's coordinate
//	p[1] += K / 2;
//
//	// the direction to traverse layers of grid. dir = 0 for x, dir = 1 for y, dir = 2 for z
//	int dir = 0;
//	int beg = 0;
//	int step = 1;
//	if (fabs(v[1]) > fabs(v[0]) && fabs(v[1]) > fabs(v[2])) {
//		dir = 1;
//	}
//	else if (fabs(v[2]) > fabs(v[0]) && fabs(v[2]) > fabs(v[1])) {
//		dir = 2;
//	}
//	if (v[dir] > 0) {
//		beg = K - 1;
//		step = -1;
//	}
//	int dir1 = dir + 1 % 3, dir2 = dir + 2 % 3;
//
//	cv::Vec3b old_C, new_C;
//	double old_S = INFINITY, new_S;
//	cv::Mat old_real_p, new_real_p;
//	for (int i = beg; i >= 0 && i < K; i++) {
//		double t;
//		cv::Vec3d p0, p1, p2;
//		Voxel* voxel[3];
//		t = (i - p[dir]) / v[dir];
//		p0[dir] = (double)i;
//		p0[dir1] = p[dir1] + t * v[dir1];
//		p0[dir2] = p[dir2] + t * v[dir2];
//		voxel[0] = getVoxelByCoordinate(p0);
//		t = (i + 1 - p[dir]) / v[dir];
//		p2[dir] = (double)i;
//		p2[dir1] = p[dir1] + t * v[dir1];
//		p2[dir2] = p[dir2] + t * v[dir2];
//		voxel[2] = getVoxelByCoordinate(p2);
//		if (min(p0[dir1], p2[dir1]) - (int)max(p0[dir1], p2[dir1]) < 0) {
//			if (min(p0[dir2], p2[dir2]) - (int)max(p0[dir2], p2[dir2]) < 0) {
//				p1[dir] = i;
//				p1[dir1] = (v[dir1] > 0) ? min(p0[dir1], p2[dir1]) : max(p0[dir1], p2[dir1]);
//				p1[dir2] = (v[dir2] > 0) ? min(p0[dir2], p2[dir2]) : max(p0[dir2], p2[dir2]);
//				voxel[1] = getVoxelByCoordinate(p1);
//			}
//		}
//		int cnt = 0;
//		new_S = 0;
//		new_C = cv::Vec3b::all(0);
//		for (int j = 0; j < 3; j++) {
//			if (voxel[j]) {
//				new_C += voxel[j]->C;
//				new_S += voxel[j]->S;
//				cnt++;
//			}
//		}
//		if (cnt == 0) {
//			continue;
//		}
//		new_S /= cnt;
//		new_C /= cnt;
//		if (new_S <= 0) {
//			cv::Vec3d tmp;
//			tmp[dir] = i + 0.5;
//			tmp[dir1] = (p0[dir1] + p2[dir1]) / 2;
//			tmp[dir2] = (p2[dir2] + p2[dir2]) / 2;
//			new_real_p = cv::Mat(tmp);
//			if (new_S == 0) {
//				C = new_C;
//			}
//			else {
//				double tmp_newS = -1 / new_S, tmp_oldS = 1 / old_S;
//				double tmp_sumS = tmp_newS + tmp_oldS;
//				double para_new = tmp_newS / tmp_sumS;
//				double para_old = tmp_oldS / tmp_sumS;
//				C = para_new * new_C + para_new * old_C;
//				new_real_p = para_new * new_real_p + para_old * old_real_p;
//			}
//			worldP = new_real_p - cv::Mat(cv::Vec3d(K / 2, K / 2, 0));
//			return true;
//		}
//		else {
//			old_C = new_C;
//			old_S = new_S;
//			old_real_p = new_real_p;
//		}
//	}
//	return false;
//}

//bool Grid::getColorAndPointByRay(cv::Point3d p, cv::Vec3d v, cv::Vec3b & C, cv::Mat & worldP) {
//	p.x += K / 2;  // move the grid corner - grid[0][0][0] to the original point
//	p.y += K / 2;
//
//	int beg = 0;
//	int step = 1;
//
//	// decide the direction of raycasting
//	if (fabs(v[0]) > fabs(v[1]) && fabs(v[0]) > fabs(v[2])) {
//		if (v[0] > 0) {
//			beg = K - 1;
//			step = -1;
//		}
//		cv::Vec3b old_C, new_C;
//		double old_S = INFINITY, new_S;
//		cv::Mat old_real_p(cv::Vec3d::all(0)), new_real_p;
//		for (int i = beg; i >= 0 && i < K; i += step) {  // iterate by layer, find S, C of this layer
//			new_S = 0;
//			new_C = cv::Vec3b::all(0);
//			double t, y0, z0, y1, z1, y2, z2;
//			Voxel *voxel0 = nullptr, *voxel1 = nullptr, *voxel2 = nullptr;
//			t = (i - p.x) / v[0];
//			y0 = p.y + t * v[1];
//			z0 = p.z + t * v[2];
//			voxel0 = getVoxelByCoordinate(i, y0, z0);
//			t = (i + 1 - p.x) / v[0];
//			y2 = p.y + t * v[1];
//			z2 = p.z + t * v[2];
//			voxel2 = getVoxelByCoordinate(i, y2, z2);
//			if (min(y0, y2) - (int)max(y0, y2) < 0) {
//				if (min(z0, z2) - (int)max(z0, z2) < 0) {
//					y1 = (v[1] > 0) ? min(y0, y2) : max(y0, y2);
//					z1 = (v[2] > 0) ? min(z0, z2) : max(z0, z2);
//					voxel1 = getVoxelByCoordinate(i, y1, z1);
//				}
//			}
//			int cnt = 0;
//			if (voxel0) {
//				new_C += voxel0->C;
//				new_S += voxel0->S;
//				cnt++;
//			}
//			if (voxel1) {
//				new_C += voxel1->C;
//				new_S += voxel1->S;
//				cnt++;
//			}
//			if (voxel2) {
//				new_C += voxel2->C;
//				new_S += voxel2->S;
//				cnt++;
//			}
//			new_C /= cnt;
//			new_S /= cnt;
//			if (new_S <= 0) {
//				new_real_p = cv::Mat(cv::Vec3d(i + 0.5, (y0 + y2) / 2, (z0 + z2) / 2));
//				if (new_S == 0) {
//					C = new_C;
//				}
//				else {
//					double tmp_newS = 1 / new_S, tmp_oldS = 1 / old_S;
//					double tmp_sumS = tmp_newS + tmp_oldS;
//					double para_new = tmp_newS / tmp_sumS;
//					double para_old = tmp_oldS / tmp_sumS;
//					C = para_new * new_C + para_new * old_C;
//					new_real_p = para_new * new_real_p + para_old * old_real_p;
//				}
//				worldP = new_real_p - cv::Mat(cv::Vec3d(K / 2, K / 2, 0));
//				return true;
//			}
//			else {
//				old_C = new_C;
//				old_S = new_S;
//				old_real_p = new_real_p;
//			}
//		}
//	}
//	else if (fabs(v[1]) > fabs(v[0]) && fabs(v[1]) > fabs(v[2])) {
//		if (v[1] > 0) {
//
//		}
//		else {
//
//		}
//	}
//	else if (fabs(v[2]) > fabs(v[0]) && fabs(v[2]) > fabs(v[1])) {
//		if (v[2] > 0) {
//
//		}
//		else {
//
//		}
//	}
//	else {
//
//	}
//	
//	
//	return false;
//}


