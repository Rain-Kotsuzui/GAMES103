using UnityEngine;
using System.Collections;
using Unity.VisualScripting;

public class wave_motion : MonoBehaviour
{
	int size = 100;
	float rate = 0.005f;
	float gamma = 0.004f;
	float damping = 0.996f;
	float cell_size = 0.1f;

	float[,] old_h;
	float[,] low_h;
	float[,] vh;
	float[,] b;

	bool[,] cg_mask;
	float[,] cg_p;
	float[,] cg_r;
	float[,] cg_Ap;
	bool tag = true;

	Vector3 cube_v = Vector3.zero;
	Vector3 cube_w = Vector3.zero;


	// Use this for initialization
	void Start()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		mesh.Clear();

		Vector3[] X = new Vector3[size * size];

		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				X[i * size + j].x = i * cell_size - size * cell_size / 2;
				X[i * size + j].y = 0;
				X[i * size + j].z = j * cell_size - size * cell_size / 2;
			}

		int[] T = new int[(size - 1) * (size - 1) * 6];
		int index = 0;
		for (int i = 0; i < size - 1; i++)
			for (int j = 0; j < size - 1; j++)
			{
				T[index * 6 + 0] = (i + 0) * size + (j + 0);
				T[index * 6 + 1] = (i + 0) * size + (j + 1);
				T[index * 6 + 2] = (i + 1) * size + (j + 1);
				T[index * 6 + 3] = (i + 0) * size + (j + 0);
				T[index * 6 + 4] = (i + 1) * size + (j + 1);
				T[index * 6 + 5] = (i + 1) * size + (j + 0);
				index++;
			}
		mesh.vertices = X;
		mesh.triangles = T;
		mesh.RecalculateNormals();

		low_h = new float[size, size];
		old_h = new float[size, size];
		vh = new float[size, size];
		b = new float[size, size];

		cg_mask = new bool[size, size];
		cg_p = new float[size, size];
		cg_r = new float[size, size];
		cg_Ap = new float[size, size];

		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				low_h[i, j] = 99999;
				old_h[i, j] = 0;
				vh[i, j] = 0;
			}
	}

	void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
	{
		for (int i = li; i <= ui; i++)
			for (int j = lj; j <= uj; j++)
				if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
				{
					Ax[i, j] = 0;
					if (i != 0) Ax[i, j] -= x[i - 1, j] - x[i, j];
					if (i != size - 1) Ax[i, j] -= x[i + 1, j] - x[i, j];
					if (j != 0) Ax[i, j] -= x[i, j - 1] - x[i, j];
					if (j != size - 1) Ax[i, j] -= x[i, j + 1] - x[i, j];
				}
	}

	float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
	{
		float ret = 0;
		for (int i = li; i <= ui; i++)
			for (int j = lj; j <= uj; j++)
				if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
				{
					ret += x[i, j] * y[i, j];
				}
		return ret;
	}

	void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
	{
		//Solve the Laplacian problem by CG.
		A_Times(mask, x, cg_r, li, ui, lj, uj);

		for (int i = li; i <= ui; i++)
			for (int j = lj; j <= uj; j++)
				if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
				{
					cg_p[i, j] = cg_r[i, j] = b[i, j] - cg_r[i, j];
				}

		float rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);

		for (int k = 0; k < 128; k++)
		{
			if (rk_norm < 1e-10f) break;
			A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
			float alpha = rk_norm / Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

			for (int i = li; i <= ui; i++)
				for (int j = lj; j <= uj; j++)
					if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
					{
						x[i, j] += alpha * cg_p[i, j];
						cg_r[i, j] -= alpha * cg_Ap[i, j];
					}

			float _rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);
			float beta = _rk_norm / rk_norm;
			rk_norm = _rk_norm;

			for (int i = li; i <= ui; i++)
				for (int j = lj; j <= uj; j++)
					if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
					{
						cg_p[i, j] = cg_r[i, j] + beta * cg_p[i, j];
					}
		}

	}

	void Shallow_Wave(float[,] old_h, float[,] h, float[,] new_h)
	{
		//Step 1:
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				new_h[i, j] = h[i, j] + (h[i, j] - old_h[i, j]) * damping;
				float h_im1_j = (i > 0) ? h[i - 1, j] : h[i, j];
				float h_ip1_j = (i < size - 1) ? h[i + 1, j] : h[i, j];
				float h_i_jm1 = (j > 0) ? h[i, j - 1] : h[i, j];
				float h_i_jp1 = (j < size - 1) ? h[i, j + 1] : h[i, j];
				float lap_h = h_im1_j + h_ip1_j + h_i_jm1 + h_i_jp1 - 4 * h[i, j];

				new_h[i, j] += rate * lap_h;
			}
		//Step 2: Block->Water coupling
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				cg_mask[i, j] = false;
				vh[i, j] = 0;
			}
		}
		int total_li = size, total_ui = -1, total_lj = size, total_uj = -1;
		System.Action<GameObject> processBlock = (block) =>
		{
			if (block == null) return;

			Bounds bounds = block.GetComponent<Collider>().bounds;
			Collider blockCollider = block.GetComponent<Collider>();

			float grid_origin_x = -size * cell_size / 2;
			float grid_origin_z = -size * cell_size / 2;

			int li = (int)((bounds.min.x - grid_origin_x) / cell_size);
			int ui = (int)((bounds.max.x - grid_origin_x) / cell_size);
			int lj = (int)((bounds.min.z - grid_origin_z) / cell_size);
			int uj = (int)((bounds.max.z - grid_origin_z) / cell_size);

			li = Mathf.Max(0, li);
			ui = Mathf.Min(size - 1, ui);
			lj = Mathf.Max(0, lj);
			uj = Mathf.Min(size - 1, uj);

			if (ui >= li && uj >= lj)
			{
				total_li = Mathf.Min(total_li, li);
				total_ui = Mathf.Max(total_ui, ui);
				total_lj = Mathf.Min(total_lj, lj);
				total_uj = Mathf.Max(total_uj, uj);

				for (int i = li; i <= ui; i++)
				{
					for (int j = lj; j <= uj; j++)
					{
						// *** MODIFICATION START ***
						// Use Raycasting to find the precise height of the (potentially rotated) block bottom.
						float world_x = grid_origin_x + (i + 0.5f) * cell_size;
						float world_z = grid_origin_z + (j + 0.5f) * cell_size;

						// Start ray from below the grid cell and cast upwards.
						Ray ray = new Ray(new Vector3(world_x, bounds.min.y - 1.0f, world_z), Vector3.up);
						RaycastHit hit;

						// Check if the ray hits THIS block.
						if (blockCollider.Raycast(ray, out hit, 100.0f) && hit.point.y < new_h[i, j])
						{
							cg_mask[i, j] = true;
							low_h[i, j] = hit.point.y; // The precise height
							b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
						}
						// If it doesn't hit, cg_mask[i,j] remains false.
						// *** MODIFICATION END ***
					}
				}
			}
		};

		processBlock(GameObject.Find("Cube"));
		processBlock(GameObject.Find("Block"));
		processBlock(GameObject.Find("Sphere"));
		if (total_ui >= total_li)
		{
			Conjugate_Gradient(cg_mask, b, vh, total_li, total_ui, total_lj, total_uj);
		}

		//TODO: Diminish vh.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				vh[i, j] *= gamma;
			}
		}
		//TODO: Update new_h by vh.
		for (int i = 0; i < size; i++)
		{
			for (int j = 0; j < size; j++)
			{
				float vh_im1_j = (i > 0) ? vh[i - 1, j] : vh[i, j];
				float vh_ip1_j = (i < size - 1) ? vh[i + 1, j] : vh[i, j];
				float vh_i_jm1 = (j > 0) ? vh[i, j - 1] : vh[i, j];
				float vh_i_jp1 = (j < size - 1) ? vh[i, j + 1] : vh[i, j];
				float lap_v = vh_im1_j + vh_ip1_j + vh_i_jm1 + vh_i_jp1 - 4 * vh[i, j];

				new_h[i, j] += lap_v * rate;
			}
		}
		//Step 3
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				old_h[i, j] = h[i, j];
				h[i, j] = new_h[i, j];
			}

		//Step 4: Water->Block coupling.
		System.Action<GameObject> applyForceToBlock = (block) =>
		{
			if (block == null) return;

			Rigidbody rb = block.GetComponent<Rigidbody>();
			if (rb == null) return;

			Bounds bounds = block.GetComponent<Collider>().bounds;

			float grid_origin_x = -size * cell_size / 2;
			float grid_origin_z = -size * cell_size / 2;

			int li = Mathf.Max(0, (int)((bounds.min.x - grid_origin_x) / cell_size));
			int ui = Mathf.Min(size - 1, (int)((bounds.max.x - grid_origin_x) / cell_size));
			int lj = Mathf.Max(0, (int)((bounds.min.z - grid_origin_z) / cell_size));
			int uj = Mathf.Min(size - 1, (int)((bounds.max.z - grid_origin_z) / cell_size));

			Vector3 total_force = Vector3.zero;
			Vector3 total_torque = Vector3.zero;

			float force_factor = 20.0f;

			for (int i = li; i <= ui; i++)
			{
				for (int j = lj; j <= uj; j++)
				{
					if (cg_mask[i, j])
					{
						Vector3 force_ij = Vector3.up * vh[i, j] * force_factor * (cell_size * cell_size);
						total_force += force_ij;

						// *** MODIFICATION START ***
						// The force application point's Y should be the actual contact point height, not bounds.min.y
						float world_x = grid_origin_x + (i + 0.5f) * cell_size;
						float world_z = grid_origin_z + (j + 0.5f) * cell_size;
						Vector3 force_position = new Vector3(world_x, low_h[i, j], world_z);
						// *** MODIFICATION END ***

						Vector3 r = force_position - rb.worldCenterOfMass;
						total_torque += Vector3.Cross(r, force_ij);
					}
				}
			}

			rb.AddForce(total_force / 8.0f);
			rb.AddTorque(total_torque / 8.0f);
		};

		applyForceToBlock(GameObject.Find("Cube"));
		applyForceToBlock(GameObject.Find("Block"));
		
		applyForceToBlock(GameObject.Find("Sphere"));
	}


	// Update is called once per frame
	void Update()
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X = mesh.vertices;
		float[,] new_h = new float[size, size];
		float[,] h = new float[size, size];
		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				h[i, j] = X[i * size + j].y;
			}
		if (Input.GetKeyDown("r"))
		{
			int i = Random.Range(1, size - 1);
			int j = Random.Range(1, size - 1);
			float r = Random.Range(0.5f, 1.0f);
			h[i, j] += r;
			h[i - 1, j] -= r / 4;
			h[i + 1, j] -= r / 4;
			h[i, j - 1] -= r / 4;
			h[i, j + 1] -= r / 4;
		}

		for (int l = 0; l < 8; l++)
		{
			Shallow_Wave(old_h, h, new_h);
		}

		for (int i = 0; i < size; i++)
			for (int j = 0; j < size; j++)
			{
				X[i * size + j].y = h[i, j];
			}
		mesh.vertices = X;
		mesh.RecalculateNormals();

	}
}