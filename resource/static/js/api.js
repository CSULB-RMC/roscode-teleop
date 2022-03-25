const baseAPIURL = `http://127.0.0.1:5000/api`;

/* Get subscriber data
 * @param subscriber the string value for the subscriber name
 * @return promise for subscriber data
 */
const toggleAttribute = async (attr) => {
  const data = await fetch(`${baseAPIURL}/toggle/${attr}`, {
    method: "POST",
  });
  return await data.text();
};

/* Get subscriber data
 * @param subscriber the string value for the subscriber name
 * @return promise for subscriber data
 */
const getSubscriberData = async (subscriber) => {
  const data = await fetch(`${baseAPIURL}/subscriber/${subscriber}`);
  return await data.text();
};

/* Get toggle status
 * @param attr the string value for the toggle name
 * @return promise for subscriber data
 */
const getToggleStatus = async (attr) => {
  const data = await fetch(`${baseAPIURL}/toggle/${attr}`);
  return await data.text();
};

export { toggleAttribute, getSubscriberData, getToggleStatus };
