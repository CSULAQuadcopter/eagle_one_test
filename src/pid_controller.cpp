while(1)
{
	current_reference = getReference();
	current_error = current_reference - output;
	total_error += current_error;
	output = P * current_error + I * current_error;
	output += D * (current_error - last_error);
	last_error = current_error;
	return output;
}