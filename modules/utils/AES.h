//
//  rijndael.h
//
//  February 2013
// 	Optimised ANSI C code for the Rijndael cipher by Gabriele Merlonghi
//
//	This code is hereby placed in the public domain.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.
//

#ifndef H__RIJNDAEL
#define H__RIJNDAEL

#include <stdint.h>

int rijndaelSetupEncrypt(unsigned long *rk, const unsigned char *key,
  int keybits);
int rijndaelSetupDecrypt(unsigned long *rk, const unsigned char *key,
  int keybits);
void rijndaelEncrypt(const unsigned long *rk, int nrounds,
  const unsigned char plaintext[16], unsigned char ciphertext[16]);
void rijndaelDecrypt(const unsigned long *rk, int nrounds,
  const unsigned char ciphertext[16], unsigned char plaintext[16]);

#define KEYLENGTH(keybits) ((keybits)/8)
#define RKLENGTH(keybits)  ((keybits)/8+28)
#define NROUNDS(keybits)   ((keybits)/32+6)

//CODE OPTIMIZATION defines
#define AES128_SUPPORT	1	/*MGCryptor: enable the support to AES 128bit key lenght*/
#define AES192_SUPPORT  1	/*MGCryptor: enable the support to AES 192bit key lenght*/
#define AES256_SUPPORT  1	/*MGCryptor: enable the support to AES 256bit key lenght*/


class AESCryptor2// : public AES
{
private:
	int nrounds;
	unsigned long e_key[64];
	unsigned long d_key[64];
public:
	void set_key(const uint8_t key[], int key_bits)
	{
		nrounds = NROUNDS(key_bits);
		rijndaelSetupEncrypt(e_key, key, key_bits);
		rijndaelSetupDecrypt(d_key, key, key_bits);
	}
	void encrypt(const uint8_t in_blk[16], uint8_t out_blk[16])
	{
		rijndaelEncrypt(e_key, nrounds, in_blk, out_blk);
	}
	void decrypt(const uint8_t in_blk[16], uint8_t out_blk[16])
	{
		rijndaelDecrypt(d_key, nrounds, in_blk, out_blk);
	}
};	 


#endif
