
typedef struct npk_data {
  float humidity;
  float temperature; 
  unsigned long int conductivity; 
  float pH;
  int Nitrogen; 
  int Phosphorus; 
  int Potassium;
  unsigned long int Salinity;
  unsigned long int TDS;
} npk_data_t;


void NPK_init(void);
void NPK_get(npk_data_t* npk_data);