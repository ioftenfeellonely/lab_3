Лабораторна робота №3 Павленко Андрій АС-31 i2c (варіант №9):

## Підключення
<img width="960" height="1280" alt="image" src="https://github.com/user-attachments/assets/e152a71c-909d-4994-804b-5a8f20113419" />


## Конфігурації пінів
<img width="1897" height="982" alt="image" src="https://github.com/user-attachments/assets/1bfa8389-ce6c-453b-bf7d-5cd48f14bafd" />


Змінні
```
static uint16_t i2c_timeout = 100;
static uint8_t  mpu_addr = 0;
volatile uint8_t  who_am_i = 0;

volatile int16_t  gyro_x = 0, gyro_y = 0, gyro_z = 0;
volatile float    gyro_dps_x = 0.0f, gyro_dps_y = 0.0f, gyro_dps_z = 0.0f;
```

I2C доступ до реєстрів
```
static HAL_StatusTypeDef mpu6500_write(uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(&hi2c2, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, i2c_timeout);
}
static HAL_StatusTypeDef mpu6500_read(uint8_t reg, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(&hi2c2, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, i2c_timeout);
}
```

Пошук сенсора + WHO_AM_I
```
static HAL_StatusTypeDef mpu6500_detect(void) {
  uint8_t id = 0;
  mpu_addr = MPU6500_ADDR_LOW;
  if (HAL_I2C_IsDeviceReady(&hi2c2, mpu_addr, 2, i2c_timeout) == HAL_OK) {
    if (mpu6500_read(REG_WHO_AM_I, &id, 1) == HAL_OK) { who_am_i = id; return HAL_OK; }
  }
  mpu_addr = MPU6500_ADDR_HIGH;
  if (HAL_I2C_IsDeviceReady(&hi2c2, mpu_addr, 2, i2c_timeout) == HAL_OK) {
    if (mpu6500_read(REG_WHO_AM_I, &id, 1) == HAL_OK) { who_am_i = id; return HAL_OK; }
  }
  return HAL_ERROR;
}
```

Конфігурація сенсора для варіанта №9
```
static HAL_StatusTypeDef mpu6500_init_variant9(void) {
  HAL_StatusTypeDef st; uint8_t v;

  st = mpu6500_write(REG_PWR_MGMT_1, 0x01); if (st != HAL_OK) return st;
  HAL_Delay(10);

  st = mpu6500_write(REG_CONFIG, 0x00);      if (st != HAL_OK) return st;

  if ((st = mpu6500_read(REG_GYRO_CONFIG, &v, 1)) != HAL_OK) return st;
  v &= (uint8_t)~0x18u; v |= (uint8_t)(1u << 3); 
  v &= (uint8_t)~0x03u; v |= (uint8_t)0x03u;     
  if ((st = mpu6500_write(REG_GYRO_CONFIG, v)) != HAL_OK) return st;

  st = mpu6500_write(REG_SMPLRT_DIV, 18);   if (st != HAL_OK) return st;

  return HAL_OK;
}
```

Читання та масштабування
```
uint8_t data[6];
if (mpu6500_read(REG_GYRO_XOUT_H, data, sizeof(data)) == HAL_OK) {
  gyro_x = (int16_t)((data[0] << 8) | data[1]);
  gyro_y = (int16_t)((data[2] << 8) | data[3]);
  gyro_z = (int16_t)((data[4] << 8) | data[5]);

  gyro_dps_x = (float)gyro_x / GYRO_SENS_500DPS;
  gyro_dps_y = (float)gyro_y / GYRO_SENS_500DPS;
  gyro_dps_z = (float)gyro_z / GYRO_SENS_500DPS;
}
```

```
function test() {
  console.log("notice the blank line before this function?");
}
```

Результат виконання та підтвердження працездатності

<img width="622" height="414" alt="image" src="https://github.com/user-attachments/assets/83e03113-d9ad-4158-9397-e24c026355b9" />

<img width="717" height="210" alt="image" src="https://github.com/user-attachments/assets/c15c1989-31a0-46ce-8667-8543d330c615" />

