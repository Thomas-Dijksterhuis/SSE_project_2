#define m 64  // Filter order
#define MU 0.1f  // Step size (mu parameter)

float w[m] = {0};
float x[m] = {0};
int idx = 0;

void setup() {
}

void nlms(float u, float d, int M, float mu, float *y, float *e) {
    x[idx] = u;

    float y_val = 0;
    float power = 0;
    int j = idx;

    for (int i = 0; i < M; i++) {
        y_val += w[i] * x[j];
        power += x[j] * x[j];

        if (--j < 0) j = M - 1;
    }

    if (power < 1e-6) power += 1e-6f;

    float e_val = d - y_val;
    float step = mu / power;

    j = idx;
    for (int i = 0; i < M; i++) {
        w[i] += step * e_val * x[j];

        if (--j < 0) j = M - 1;
    }
    idx = (idx + 1) % M;

    if (y) *y = y_val;
    if (e) *e = e_val;
}


void loop() {

    float u = 0;
    float d = 0;
    float *output_cleaned;
    float *output_error;
    nlms(u, d, m, MU, output_cleaned, output_error);

    Serial.printf("%f\n", *output_cleaned);
}
